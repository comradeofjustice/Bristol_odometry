# 03 — Effective wheel separation (Odometry Exercise 3)

Firmware `Rotate/` turns the robot on the spot by `TARGET_ANGLE_DEG`
(encoder closed loop, left + right counts held at 0 so the centre stays put; + = anticlockwise).
The heading change reported by odometry (r = 16.36 mm, B_old = 90 mm) is compared with the
turn measured on the printed radial marker. Because Δθ = (s_R − s_L) / B,

B_new = B_old · Δθ_est / Δθ_actual.

±90° and ±180° are used for calibration (least squares through the origin, Δθ_est = k · Δθ_actual,
B = B_old · k); ±270° is held back for evaluation.

| Commanded | Role | Measured (deg) | Odometry, B = 90 (deg) | Single-trial B (mm) |
|---|---|---|---|---|
| +90 | calibration | +100 | +90.59 | 81.53 |
| +180 | calibration | +195 | +180.27 | 83.20 |
| −90 | calibration | −98 | −91.50 | 84.03 |
| −180 | calibration | −193 | −180.45 | 84.15 |
| +270 | held-back | +285 | +271.04 | 85.59 |
| −270 | held-back | −290 | −270.67 | 84.00 |

| | Value |
|---|---|
| Slope k = Δθ_est / Δθ_actual | **0.9276** (± 0.0051) |
| Effective separation B = 90 · k | **83.48 mm** (± 0.46 mm) |
| Single-trial B, calibration set | mean 83.23, median 83.62, sd 1.21 mm |
| Held-back ±270°: mean heading error | **16.6° with B = 90 → 4.5° with B = 83.48** (+270: −7.2°, −270: +1.8°) |

![Rotation fit](analysis/rotation_fit.png)

The notebook `analysis/odometry_exercise03_wheel_separation_CSY.ipynb` was run end to end in
Google Colab (it clones this repository for its data):
[open in Colab](https://colab.research.google.com/github/comradeofjustice/Bristol_odometry/blob/main/03_wheel_separation/analysis/odometry_exercise03_wheel_separation_CSY.ipynb).

Notes for discussion:

- The robot always turned further than odometry reported, so the 90 mm in the model was too large.
- Clockwise single-trial values agree closely (84.0–84.2 mm); anticlockwise ones spread more (81.5–85.6 mm).
- Odometry puts the centre point within 3 mm of the start throughout every turn.
- Only one physical reading per condition so far (the brief asks for ≥ 5), so the ± values are optimistic.
- Sensitivity to reading the protractor: if the +180° reading (195°) had been 185°, the fit would give B = 85.18 mm.
- `measurements.csv` also lists three logged runs with no physical reading
  (`*_extra.csv`: earlier +180, −90, −180 attempts); they are kept as raw data and excluded from the fit.
- Effective value: it depends on r, the surface, coasting after the motors stop and the measurement method.

## Files

| Path | What it is |
|---|---|
| `Rotate/` | Sketch: on-the-spot turn by `TARGET_ANGLE_DEG` (+ = anticlockwise), left + right counts held at 0, persistent trial counter; `wifi_logger.py` names files `rot_<angle>deg_t<trial>_*.csv`. |
| `data/*.csv` | Raw turn logs named by commanded angle (`90deg.csv`, `-270deg.csv`; `*_extra.csv` = runs without a physical reading). |
| `measurements.csv` | Measured turn per log, role (calibration / held-back / not measured), notes. |
| `analysis/` | Notebook with outputs (run in Colab) and `rotation_fit.png`. |

Board setup: Arduino IDE, M5Stack board package, board **M5StampC3**, USB CDC On Boot **Disabled**, upload speed 115200.
