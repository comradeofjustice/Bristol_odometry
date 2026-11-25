# Odometry Exercise 2 — Effective wheel radius (Pololu 3Pi+ / M5Stack StampC3)

SEMTM0043/42 Robotics Science & Systems, University of Bristol.
All lengths are in millimetres.

## Result

Least-squares fit through the origin of **reported odometry displacement** against
**measured (real) displacement** over five trials:

| | Value |
|---|---|
| Slope k = reported / real | **0.9780** (± 0.0079), R² = 0.9997 |
| Effective wheel radius r = 16.0 / k | **16.36 mm** (± 0.13 mm) |
| Fit with intercept, y = a·x + b | a = 0.9772, b = 1.67 mm → r = 16.37 mm |
| Forward trials only | r = 16.12 mm |
| Reverse trials only | r = 16.60 mm |

![Least-squares fit](analysis/least_squares_fit.png)

Why the slope gives the radius: for the same encoder count n,
reported = (2π · r_standard / N) · n and real = (2π · r_true / N) · n, so
reported = (r_standard / r_true) · real. The slope is k = r_standard / r_true, hence
r_true = r_standard / k, with r_standard = 16.0 mm and N = 358.3 counts per revolution.

## Trials

| Trial | CSV | Command (mm) | Measured (mm) | Reported (mm) | Single-trial r (mm) |
|---|---|---|---|---|---|
| 1 | `data/5cm.csv` | +50 | +52 | +50.78 | 16.39 |
| 2 | `data/10cm.csv` | +100 | +103 | +101.00 | 16.32 |
| 3 | `data/18cm.csv` | +180 | +181 | +180.53 | 16.04 |
| 4 | `data/-10cm.csv` | −100 | −106 | −101.13 | 16.77 |
| 5 | `data/-18cm.csv` | −180 | −187 | −180.81 | 16.55 |

"Reported" is the local odometry displacement: the logged encoder counts replayed through
the same first-order Euler model as `OdometryModel.cpp` (r = 16.0 mm, B = 90.0 mm).
"Measured" is the ruler measurement of the same chassis reference point along the printed x axis.

Notes for discussion:

- Only one trial per condition, so the ± values describe the scatter of these five points only.
- Reverse trials travelled 5–6 mm further than reported, forward trials 0.5–2 mm. This suggests
  a direction-dependent effect (slip, start/stop behaviour or measurement), worth more repeats.
- The result is an effective radius for this surface and load, not necessarily the geometric
  wheel radius.

## Repository contents

| Path | What it is |
|---|---|
| `Go_Straight/` | Arduino sketch for the StampC3. Drives `TARGET_DISTANCE_MM` in a straight line (encoder-synchronised wheels, closed-loop stop), logs CSV over Wi-Fi and USB, keeps the run in RAM and resends it if the laptop connects late. |
| `wifi_logger.py` | Laptop logger (Python standard library only). Saves one CSV per run, requests a resend after late connection, never saves a run twice. |
| `data/*.csv` | Raw logs: `run,time_ms,left_pwm,right_pwm,left_counts,right_counts,diff` (counts since start of run, 10 ms rows: 0.5 s still, drive, 1 s after motors off). |
| `data/measurements.csv` | Ruler measurements (编号, CSV, 方向, 实际距离 mm). |
| `analysis/odometry_exercise02_wheel_radius_CSY.ipynb` | Colab notebook (based on the course notebook) with outputs. Reads the Google Sheet and the CSVs from Google Drive; set `USE_EXAMPLE_DATA = True` to run on synthetic data. |
| `analysis/least_squares_fit.png` | The final least-squares plot. |

Board setup: Arduino IDE, M5Stack board package, board **M5StampC3**, USB CDC On Boot **Disabled**,
upload speed 115200.
