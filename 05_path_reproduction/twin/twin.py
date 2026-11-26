"""Digital twin of the Path_Reproduction run (Pololu 3Pi+ with StampC3).

The twin runs the SAME controller as ../Path_Reproduction/Path_Reproduction.ino
(same path table, same constants, same 10 ms control step, integer encoder
counts) on a simulated robot whose motor and geometry model can be chosen:

  MODELS["course"]      the supplied Processing Digital Twin's assumptions:
                        PWM -> wheel speed linear (PWM 200 = 0.25 mm/ms) for
                        both wheels, no deadband, no lag, B = 90 mm, r = 16 mm.
  MODELS["calibrated"]  fitted to this robot's logs (Ex2-Ex4): per-wheel gain
                        above a deadband, first-order lag, right-wheel command
                        delay, B = 83.48 mm, r = 16.36 mm.
  MODELS["refit_t6"]    the calibrated model with motor gains re-fitted on the real
                        path run path_t6 (wooden floor, new batteries).
All models truncate PWM to an integer, as the robot's Robot.cpp does.

run(model) returns one row per 20 ms log period with the same columns as the
robot's CSV (time_ms, segment, left_pwm, right_pwm, left_counts, right_counts,
x_mm, y_mm, theta_rad = the robot's own encoder odometry) plus the twin's
TRUE pose (x_true_mm, y_true_mm, theta_true_rad).

Usage:  python3 twin.py            -> writes twin_course.csv and twin_calibrated.csv
"""
import math
from dataclasses import dataclass, field

import pandas as pd

# ---------------- path: keep identical to Path_Reproduction.ino ----------------
STRAIGHT, ROTATE, ARC = 0, 1, 2
PATH = [  # (type, a, radius, max_pwm)  a: straight mm / rotate-arc deg (+ = CCW)
    (STRAIGHT, 200.0, 0.0, 30.0),
    (ARC, 180.0, 80.0, 35.0),
    (STRAIGHT, 100.0, 0.0, 50.0),
    (ROTATE, -90.0, 0.0, 35.0),
    (STRAIGHT, -100.0, 0.0, 35.0),
    (ROTATE, 90.0, 0.0, 35.0),
    (STRAIGHT, 100.0, 0.0, 40.0),
    (ARC, 90.0, 60.0, 35.0),
    (ROTATE, 90.0, 0.0, 35.0),
    (STRAIGHT, 60.0, 0.0, 30.0),
]
SEGMENT_NAMES = ["straight 200 slow", "arc L R80 180", "straight 100 fast", "rotate CW 90",
                 "reverse 100", "rotate CCW 90", "straight 100", "arc L R60 90 (tight)",
                 "rotate CCW 90", "straight 60 slow"]
SEGMENT_KIND = ["straight", "arc", "straight", "rotate", "reverse", "rotate", "straight", "arc",
                "rotate", "straight"]

# controller (robot firmware) constants
CTRL_R, CTRL_B, CPR = 16.36, 83.48, 358.3
MIN_PWM, FF_DEADBAND, K_DIST, K_SYNC, MAX_CORRECTION = 25.0, 15.0, 0.3, 0.5, 10.0
RAMP_MS, SETTLE_MS, CHECKPOINT_PAUSE_MS, SEGMENT_TIMEOUT_MS = 200, 400, 0, 10000
PRE_ROLL_MS, POST_ROLL_MS, CONTROL_MS, LOG_MS = 500, 1000, 10, 20


@dataclass
class Model:
    name: str
    k_left: float            # mm/s per PWM above the deadband
    k_right: float
    deadband: float          # |PWM| below which a wheel does not turn
    tau_ms: float            # first-order motor lag
    right_delay_ms: int      # right-wheel command delay
    B: float                 # physical wheel separation, mm
    r: float                 # physical wheel radius (encoder mm per count), mm
    note: str = ""


MODELS = {
    "course": Model("course", 1.25, 1.25, 0.0, 1.0, 0, 90.0, 16.0,
                    "supplied Processing twin: linear, ideal, B=90, r=16"),
    "calibrated": Model("calibrated", 3.992, 3.117, 14.9, 5.0, 20, 83.48, 16.36,
                        "fitted to Ex2-Ex4 logs (open-loop PWM replay)"),
    "refit_t6": Model("refit_t6", 4.567, 3.370, 14.1, 5.0, 20, 83.48, 16.36,
                      "motor gains re-fitted on real run path_t6 (wooden floor, new batteries);"
                      " fitted and judged on the same run - validate on another run"),
}


def wheel_distances(seg, B=CTRL_B):
    t, a, radius, _ = seg
    if t == STRAIGHT:
        return a, a
    ang = math.radians(a)
    arc = abs(ang) * radius if t == ARC else 0.0
    return arc - ang * B / 2.0, arc + ang * B / 2.0


@dataclass
class Robot:
    model: Model
    x: float = 0.0
    y: float = 0.0
    th: float = 0.0
    nl: float = 0.0          # encoder counts (continuous)
    nr: float = 0.0
    vl: float = 0.0          # wheel speeds mm/s
    vr: float = 0.0
    pwm_l: float = 0.0
    pwm_r: float = 0.0
    queue_r: list = field(default_factory=list)

    def target(self, u, k):
        a = abs(u)
        return math.copysign(k * (a - self.model.deadband), u) if a > self.model.deadband else 0.0

    def step_1ms(self):
        m = self.model
        # The robot sends PWM to the motor board as int16: the fraction is
        # truncated toward zero (Robot.cpp: (int16_t)l). Applied in both models.
        u_l = float(math.trunc(self.pwm_l))
        self.queue_r.append(float(math.trunc(self.pwm_r)))
        if len(self.queue_r) > m.right_delay_ms + 1:
            self.queue_r.pop(0)
        u_r = self.queue_r[0] if len(self.queue_r) > m.right_delay_ms else 0.0
        self.vl += (self.target(u_l, m.k_left) - self.vl) / m.tau_ms
        self.vr += (self.target(u_r, m.k_right) - self.vr) / m.tau_ms
        sl, sr = self.vl / 1000.0, self.vr / 1000.0
        mmc = 2 * math.pi * m.r / CPR
        self.nl += sl / mmc
        self.nr += sr / mmc
        ds, dth = (sl + sr) / 2.0, (sr - sl) / m.B
        # Euler update, as the supplied OdometryModel.pde (1 ms steps)
        self.x += ds * math.cos(self.th)
        self.y += ds * math.sin(self.th)
        self.th += dth


def run(model="calibrated", path=PATH, checkpoint_pause_ms=CHECKPOINT_PAUSE_MS):
    m = MODELS[model] if isinstance(model, str) else model
    bot = Robot(m)
    mmc = 2 * math.pi * CTRL_R / CPR
    ox = oy = oth = 0.0
    prev_l = prev_r = 0
    state, seg, seg_label = "PRE_ROLL", 0, 0
    seg_t0 = settle_t0 = off_t = 0
    seg_l0 = seg_r0 = 0
    tl = tr = 0.0
    rows, t, last_log = [], 0, -LOG_MS

    def start_segment(now, lc, rc):
        nonlocal seg_l0, seg_r0, seg_t0, tl, tr, seg_label, state
        sl, sr = wheel_distances(path[seg])
        tl, tr = sl / mmc, sr / mmc
        seg_l0, seg_r0, seg_t0, seg_label, state = lc, rc, now, seg + 1, "MOVING"

    while True:
        lc, rc = math.floor(bot.nl), math.floor(bot.nr)          # integer counts
        sl_, sr_ = (lc - prev_l) * mmc, (rc - prev_r) * mmc        # robot's own odometry
        prev_l, prev_r = lc, rc
        ds = 0.5 * (sl_ + sr_)
        ox += ds * math.cos(oth)
        oy += ds * math.sin(oth)
        oth += (sr_ - sl_) / CTRL_B

        if state == "PRE_ROLL" and t >= PRE_ROLL_MS:
            start_segment(t, lc, rc)
        elif state == "MOVING":
            _, _, _, max_pwm = path[seg]
            dl, dr = lc - seg_l0, rc - seg_r0
            al, ar = abs(tl), abs(tr)
            tmax = max(al, ar)
            pl = dl / tl if al > 0.5 else 1.0
            pr = dr / tr if ar > 0.5 else 1.0
            p = (al * pl + ar * pr) / (al + ar)
            if p >= 1.0:
                bot.pwm_l = bot.pwm_r = 0.0
                state, settle_t0 = "SETTLING", t
            elif t - seg_t0 >= SEGMENT_TIMEOUT_MS:
                bot.pwm_l = bot.pwm_r = 0.0
                state, off_t, seg_label = "POST_ROLL", t, len(path) + 1
            else:
                base = min(max(K_DIST * (1.0 - p) * tmax, MIN_PWM), max_pwm)
                if t - seg_t0 < RAMP_MS:
                    base = MIN_PWM + (base - MIN_PWM) * (t - seg_t0) / RAMP_MS
                mag_l = FF_DEADBAND + (base - FF_DEADBAND) * al / tmax
                mag_r = FF_DEADBAND + (base - FF_DEADBAND) * ar / tmax
                corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, K_SYNC * (pl - pr) * tmax))
                bot.pwm_l = math.copysign(1.0, tl) * (mag_l - corr)
                bot.pwm_r = math.copysign(1.0, tr) * (mag_r + corr)
        elif state == "SETTLING" and t - settle_t0 >= SETTLE_MS + checkpoint_pause_ms:
            seg += 1
            if seg >= len(path):
                state, off_t, seg_label = "POST_ROLL", t, len(path) + 1
            else:
                start_segment(t, lc, rc)

        finished = state == "POST_ROLL" and t - off_t >= POST_ROLL_MS
        if t - last_log >= LOG_MS or finished:
            last_log = t
            rows.append((t, seg_label, round(bot.pwm_l, 1), round(bot.pwm_r, 1), lc, rc,
                         ox, oy, oth, bot.x, bot.y, bot.th))
        if finished or t > 120000:
            break
        for _ in range(CONTROL_MS):
            bot.step_1ms()
        t += CONTROL_MS

    return pd.DataFrame(rows, columns=["time_ms", "segment", "left_pwm", "right_pwm",
                                       "left_counts", "right_counts", "x_mm", "y_mm", "theta_rad",
                                       "x_true_mm", "y_true_mm", "theta_true_rad"])


def nominal_path(step_mm=2.0, path=PATH):
    """The designed path itself (exact geometry, B = controller B)."""
    x = y = th = 0.0
    pts = [(0, x, y, th)]
    for i, seg in enumerate(path):
        sl, sr = wheel_distances(seg)
        n = max(2, int(max(abs(sl), abs(sr)) / step_mm))
        for _ in range(n):
            a, b = sl / n, sr / n
            ds, dth = (a + b) / 2, (b - a) / CTRL_B
            x += ds * math.cos(th + dth / 2)
            y += ds * math.sin(th + dth / 2)
            th += dth
            pts.append((i + 1, x, y, th))
    return pd.DataFrame(pts, columns=["segment", "x_mm", "y_mm", "theta_rad"])


if __name__ == "__main__":
    for name in MODELS:
        df = run(name)
        df.to_csv(f"twin_{name}.csv", index=False)
        last = df.iloc[-1]
        print(f"{name:10s} {len(df)} rows, {last.time_ms/1000:.1f} s, true end "
              f"({last.x_true_mm:.1f}, {last.y_true_mm:.1f}, {math.degrees(last.theta_true_rad):.1f} deg), "
              f"odometry end ({last.x_mm:.1f}, {last.y_mm:.1f})")
