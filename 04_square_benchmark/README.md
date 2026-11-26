# 04 — Bidirectional rectangle benchmark (Odometry Exercise 4, in progress)

Firmware `System_error/` drives a closed 321 × 217 mm rectangle from the bottom-left corner
(`LOOP_DIRECTION` = +1 anticlockwise, −1 clockwise) with encoder closed-loop sides and on-the-spot turns,
using the frozen calibrated geometry r = 16.36 mm and B = 83.48 mm. No line following. The robot logs odometry
every 20 ms in RAM and sends the record over Wi-Fi only after it has stopped, so Wi-Fi cannot stall the control loop.

The robot centre's end position was measured on the sheet (right = +, up = +) and converted to the start frame
(x = initial heading, y = robot's left). Residual e = physical closure − odometry closure.

| Direction | Physical closure (x, y) mm | Odometry closure (x, y) mm, heading | Residual (x, y) mm | \|e\| |
|---|---|---|---|---|
| anticlockwise | (−26, −11) | (4.5, −4.7), +1.9° | (−30.4, −6.3) | 31.1 mm |
| clockwise | (−6, −26) | (18.0, 7.1), −6.8° | (−24.0, −33.1) | 40.9 mm |

![Odometry paths](analysis/odometry_paths.png)

Notes: one trial per direction so far (the brief asks for ≥ 5 each) and no final heading measured yet, so
systematic and random errors cannot be separated yet. In both trials the wheels nearly stalled for a while on
the top edge of the rectangle (see `measurements_sheet_frame.csv`). The analysis notebook
`analysis/odometry_exercise04_square_benchmark_CSY.ipynb` is the course notebook with only the data
cells changed (plus `sort=False` so the paths plot in time order):
[open in Colab](https://colab.research.google.com/github/comradeofjustice/Bristol_odometry/blob/main/04_square_benchmark/analysis/odometry_exercise04_square_benchmark_CSY.ipynb).

## Files

| Path | What it is |
|---|---|
| `System_error/` | Sketch: closed 321 × 217 mm rectangle, `LOOP_DIRECTION` +1 CCW / −1 CW, record sent after the run; logger names files `sq_<dir>_t<trial>_*.csv`. |
| `data/` | `ccw_trial1.csv`, `cw_trial1.csv` raw logs and `odometry_exercise04_square_trials.csv` in the course notebook's format. |
| `measurements_sheet_frame.csv` | Measured end displacement on the sheet (right = +, up = +), geometry, notes per trial. |
| `analysis/` | The course Ex4 notebook adapted to this data (run in Colab) and its figures. |

Board setup: Arduino IDE, M5Stack board package, board **M5StampC3**, USB CDC On Boot **Disabled**, upload speed 115200.
