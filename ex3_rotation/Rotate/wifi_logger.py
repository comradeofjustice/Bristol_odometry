#!/usr/bin/env python3
"""Live Wi-Fi CSV logger for Rotate (Python standard library only).

1. Run:   python3 wifi_logger.py
   Leave it running for the whole session - it keeps retrying until the
   robot's Wi-Fi (CSY / csy12345) is there, and reconnects after every
   power-up.
2. Switch the robot on (On/Off) and join the laptop to Wi-Fi "CSY" within
   the 10 s countdown. The robot then logs 0.5 s standing still, turns,
   and logs 1 s more after the motors stop.
3. EVERY RUN IS SAVED AS ITS OWN CSV, created when the robot sends
   "# START", named after the angle and trial number the robot reports:
       logs/rot_+90deg_t1_<date>_<time>.csv,  logs/rot_-180deg_t3_<date>_<time>.csv
   Switch off/on for the next trial; the logger makes the next file.
4. Ctrl+C when you have finished all trials.

Files are saved in the "logs" folder NEXT TO THIS SCRIPT
(Rotate/logs/), whatever folder the terminal is in.
CSV columns: run,time_ms,left_pwm,right_pwm,left_counts,right_counts,diff
'#' status lines (countdown, START, MOTORS_OFF, STOP, link drops) go to
logs/session_<date>_<time>_events.txt, so each CSV stays clean.
Optional prefix:  python3 wifi_logger.py --prefix test  ->  logs/test_<date>_<time>.csv
The '# STOP' line shows theta_est_deg = the odometry estimate of the turn.

Late connection is fine: the robot keeps the whole run in memory and sends
it again ("replay") when the laptop connects after the run, or joined
part-way through. The logger asks for this on every connection. A run that
is already saved completely is not saved twice (the robot sends a random
boot id + run number with every run). A file cut short by a lost link is
renamed *_INCOMPLETE.csv, and the replay is then saved as a complete file.
"""
import argparse
import os
import re
import socket
import sys
import time

HEADER = "run,time_ms,left_pwm,right_pwm,left_counts,right_counts,diff"
N_FIELDS = len(HEADER.split(","))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prefix", default=None, help="start of each CSV file name (default: rot_<angle>deg_t<trial>)")
    ap.add_argument("--host", default="192.168.4.1", help="robot address (ESP32 soft-AP default)")
    ap.add_argument("--port", type=int, default=80)
    a = ap.parse_args()

    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(logs_dir, exist_ok=True)
    events_path = os.path.join(logs_dir, time.strftime("session_%Y%m%d_%H%M%S_events.txt"))
    ev_f = open(events_path, "a", buffering=1)
    ev_f.write(f"# logger start {time.strftime('%Y-%m-%d %H:%M:%S')} {a.host}:{a.port}\n")
    print(f"CSV files -> {logs_dir}/{a.prefix or 'rot_<angle>deg_t<trial>'}_<date>_<time>.csv (one per run)")
    print(f"events    -> {events_path}\n(Ctrl+C to stop)")

    state = {"csv": None, "path": None, "rows": 0, "max_diff": 0, "last": None,
             "key": None, "skipping": False}
    saved = []
    completed = set()      # (boot, run) already saved completely

    def run_key(line):
        m = re.search(r"boot=(\w+)\s+run=(\d+)", line)
        return (m.group(1), m.group(2)) if m else None

    def close_incomplete(reason):
        if state["csv"]:
            state["csv"].close()
            new_path = state["path"][:-4] + "_INCOMPLETE.csv"
            os.replace(state["path"], new_path)
            msg = (f"INCOMPLETE ({reason}): {os.path.basename(new_path)} - "
                   "the robot will resend the full run when you reconnect")
            print("\n" + msg)
            ev_f.write(f"# {msg}\n")
        state.update(csv=None, path=None, rows=0, max_diff=0, last=None, key=None)

    def close_current():
        if state["csv"]:
            state["csv"].close()
            if state["key"]:
                completed.add(state["key"])
            last = state["last"]
            msg = f"saved {state['rows']} rows -> {os.path.basename(state['path'])}"
            if last:
                msg += (f" | final counts L/R {last[4]}/{last[5]}, final diff {last[6]},"
                        f" max |diff| {state['max_diff']}")
            print("\n" + msg)
            ev_f.write(f"# {msg}\n")
            saved.append(state["path"])
        state.update(csv=None, path=None, rows=0, max_diff=0, last=None, key=None)

    def name_for(start_line):
        if a.prefix:
            return a.prefix
        ang = re.search(r"angle_deg=(-?[\d.]+)", start_line)
        tri = re.search(r"trial=(\d+)", start_line)
        if ang and tri:
            return f"rot_{float(ang.group(1)):+.0f}deg_t{tri.group(1)}"
        return "rot"

    def open_new(key, start_line=""):
        if state["csv"]:
            close_incomplete("a new START arrived")
        prefix = name_for(start_line)
        path = os.path.join(logs_dir, f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        n = 2
        while os.path.exists(path):
            path = os.path.join(logs_dir, f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}_{n}.csv")
            n += 1
        state["csv"] = open(path, "w", buffering=1, newline="")
        state["csv"].write(HEADER + "\n")
        state["path"] = path
        state["key"] = key
        print(f"\nnew trial -> {path}")

    last_view = 0.0
    try:
        while True:
            try:
                with socket.create_connection((a.host, a.port), timeout=5) as s:
                    s.settimeout(None)
                    print(f"\nconnected to {a.host}:{a.port} - waiting for the robot "
                          "(a finished run will be resent automatically)")
                    ev_f.write(f"{time.strftime('%H:%M:%S')} # connected\n")
                    s.sendall(b"R\n")           # ask the robot to resend a finished run
                    buf = b""
                    while True:
                        chunk = s.recv(4096)
                        if not chunk:
                            raise ConnectionError("closed by robot")
                        buf += chunk
                        *lines, buf = buf.split(b"\n")
                        for raw in lines:
                            line = raw.decode("utf-8", errors="replace").strip()
                            if not line or line.startswith("run,"):
                                continue                      # header from the robot
                            if line.startswith("#"):
                                ev_f.write(f"{time.strftime('%H:%M:%S')} {line}\n")
                                if line.startswith("# START"):
                                    key = run_key(line)
                                    if key and key in completed:
                                        state["skipping"] = True
                                        print(f"\n(resend of boot={key[0]} run={key[1]} - already saved, ignored)", end="")
                                        continue
                                    state["skipping"] = False
                                    open_new(key, line)
                                    if "replay=1" in line:
                                        print("\n(resent by the robot after the run)", end="")
                                if state["skipping"]:
                                    if line.startswith("# STOP"):
                                        state["skipping"] = False
                                    continue
                                print("\n" + line, end="")
                                if line.startswith("# STOP"):
                                    close_current()
                                continue
                            parts = line.split(",")
                            if len(parts) != N_FIELDS:
                                ev_f.write(f"# malformed line skipped: {line}\n")
                                continue
                            if state["skipping"]:
                                continue
                            if state["csv"] is None:
                                ev_f.write(f"# row before START (not saved): {line}\n")
                                continue
                            state["csv"].write(line + "\n")
                            state["rows"] += 1
                            state["max_diff"] = max(state["max_diff"], abs(int(parts[6])))
                            state["last"] = parts
                            now = time.time()
                            if now - last_view > 0.2:
                                last_view = now
                                run, t, lp, rp, lc, rc, d = parts
                                sys.stdout.write(f"\rt={int(t)/1000:5.2f}s  PWM L/R {float(lp):5.1f}/{float(rp):5.1f}  "
                                                 f"counts L/R {lc:>6}/{rc:>6}  diff {d:>4}   ")
                                sys.stdout.flush()
            except (OSError, ConnectionError) as e:
                state["skipping"] = False
                if state["csv"]:
                    close_incomplete(f"link lost: {e}")
                ev_f.write(f"# not connected ({e}) at {time.strftime('%H:%M:%S')}\n")
                print(f"\rnot connected ({e}) - is the laptop on the robot's Wi-Fi (CSY)? retrying...   ", end="")
                time.sleep(1)
    except KeyboardInterrupt:
        if state["csv"]:
            close_incomplete("logger stopped")
        print(f"\nstopped - {len(saved)} trial file(s) saved in {logs_dir}")
        for p in saved:
            print("  ", os.path.basename(p))
    finally:
        ev_f.close()


if __name__ == "__main__":
    main()
