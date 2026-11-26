# 01 — Go Straight (encoder-synchronised straight driving + Wi-Fi logging)

The first program and the base of every later sketch.

| Path | What it is |
|---|---|
| `Go_Straight/` | Arduino sketch for the StampC3. Drives `TARGET_DISTANCE_MM` (signed) in a straight line: both wheels get the same base PWM, nudged by `K_SYNC × (left − right counts)` so the counts stay equal; slows down near the target (`K_DIST × remaining counts`, limited to 25–40 PWM) and stops on the average encoder count. Starts by itself 10 s after power-up (no button). |
| `wifi_logger.py` | Laptop logger (Python standard library only). Joins the robot's access point (CSY), saves one CSV per run, asks for a resend after a late connection and never saves a run twice. |

CSV columns: `run,time_ms,left_pwm,right_pwm,left_counts,right_counts,diff` — 10 ms rows: 0.5 s still, drive, 1 s
after the motors stop. Every row is also kept in RAM and replayed when a laptop connects after the run.

Used for the straight-line trials of [02 — wheel radius](../02_wheel_radius/).

Board setup: Arduino IDE, M5Stack board package, board **M5StampC3**, USB CDC On Boot **Disabled**, upload speed 115200.
