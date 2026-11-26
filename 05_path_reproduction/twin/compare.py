"""Comparison helpers for the Path Reproduction project (real robot vs twin).

All runs are DataFrames with the robot CSV columns (time_ms, segment,
left_counts, right_counts, x_mm, y_mm, theta_rad). Twin runs also carry the
twin's true pose (x_true_mm, y_true_mm, theta_true_rad). For the real robot the
only continuous pose is its own encoder odometry (x_mm, y_mm, theta_rad);
physical positions come from checkpoint measurements (see measurements CSV).
"""
import math

import numpy as np
import pandas as pd


def wrap_deg(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def align_by_time(a, b, cols_a=("x_mm", "y_mm", "theta_rad"), cols_b=("x_mm", "y_mm", "theta_rad")):
    """Pose difference b - a at the same time since the start of the run."""
    t = np.arange(0, min(a.time_ms.iloc[-1], b.time_ms.iloc[-1]) + 1, 20)
    out = pd.DataFrame({"time_ms": t})
    for name, df, cols in (("a", a, cols_a), ("b", b, cols_b)):
        for c, k in zip(cols, ("x", "y", "th")):
            out[f"{k}_{name}"] = np.interp(t, df.time_ms, df[c])
        out[f"seg_{name}"] = np.interp(t, df.time_ms, df.segment).round().astype(int)
    out["dpos_mm"] = np.hypot(out.x_b - out.x_a, out.y_b - out.y_a)
    out["dth_deg"] = wrap_deg(np.degrees(out.th_b - out.th_a))
    return out


def align_by_progress(a, b, cols_a=("x_mm", "y_mm", "theta_rad"), cols_b=("x_mm", "y_mm", "theta_rad"),
                      n_per_segment=40):
    """Pose difference at the same fraction of each segment (removes timing differences).
    Progress inside a segment = wheel travel since its start / total wheel travel of that segment."""
    rows = []
    for seg in sorted(set(a.segment) & set(b.segment)):
        if seg < 1:
            continue
        pieces = []
        for df, cols in ((a, cols_a), (b, cols_b)):
            g = df[df.segment == seg]
            if len(g) < 2:
                break
            travel = np.abs(np.diff(g.left_counts, prepend=g.left_counts.iloc[0])) + \
                np.abs(np.diff(g.right_counts, prepend=g.right_counts.iloc[0]))
            prog = np.cumsum(travel)
            prog = prog / prog[-1] if prog[-1] > 0 else np.linspace(0, 1, len(g))
            prog, idx = np.unique(prog, return_index=True)
            f = np.linspace(0, 1, n_per_segment)
            pieces.append([np.interp(f, prog, g[c].values[idx]) for c in cols])
        if len(pieces) < 2:
            continue
        for k in range(n_per_segment):
            rows.append({"segment": seg, "fraction": k / (n_per_segment - 1),
                         "x_a": pieces[0][0][k], "y_a": pieces[0][1][k], "th_a": pieces[0][2][k],
                         "x_b": pieces[1][0][k], "y_b": pieces[1][1][k], "th_b": pieces[1][2][k]})
    out = pd.DataFrame(rows)
    out["dpos_mm"] = np.hypot(out.x_b - out.x_a, out.y_b - out.y_a)
    out["dth_deg"] = wrap_deg(np.degrees(out.th_b - out.th_a))
    out["s"] = out.segment - 1 + out.fraction          # 0..N along the path
    return out


def segment_local_motion(df, cols=("x_mm", "y_mm", "theta_rad")):
    """Displacement and rotation of each segment in its own start frame, plus duration."""
    rows = []
    prev = df[df.segment == 0].iloc[-1] if (df.segment == 0).any() else df.iloc[0]
    for seg in sorted(s for s in df.segment.unique() if s >= 1):
        g = df[df.segment == seg]
        end = g.iloc[-1]
        x0, y0, t0 = prev[cols[0]], prev[cols[1]], prev[cols[2]]
        dx, dy = end[cols[0]] - x0, end[cols[1]] - y0
        fwd = dx * math.cos(t0) + dy * math.sin(t0)
        left = -dx * math.sin(t0) + dy * math.cos(t0)
        moving = g[(g.left_pwm != 0) | (g.right_pwm != 0)] if "left_pwm" in g else g
        dur = (moving.time_ms.iloc[-1] - moving.time_ms.iloc[0]) if len(moving) > 1 else 0
        rows.append({"segment": seg, "forward_mm": fwd, "left_mm": left,
                     "rotation_deg": math.degrees(end[cols[2]] - t0), "duration_ms": dur})
        prev = end
    return pd.DataFrame(rows)


def divergence_onset(aligned, pos_mm=5.0, heading_deg=2.0):
    """First time/segment where the difference exceeds a threshold."""
    out = {}
    for col, thr, name in (("dpos_mm", pos_mm, "position"), ("dth_deg", heading_deg, "heading")):
        hit = aligned[np.abs(aligned[col]) > thr]
        key = "time_ms" if "time_ms" in aligned else "s"
        out[name] = (None if hit.empty else
                     {"threshold": thr, key: float(hit[key].iloc[0]),
                      "segment": int(hit["seg_a"].iloc[0] if "seg_a" in hit else hit["segment"].iloc[0])})
    return out
