# Bristol odometry — Pololu 3Pi+ / M5Stack StampC3

SEMTM0043/42 Robotics Science & Systems, University of Bristol. All lengths in millimetres.
One folder per project; each has its own README, firmware, data and analysis notebook.

| # | Project | What it does | Key result |
|---|---|---|---|
| [01](01_go_straight/) | **Go Straight** | Encoder-synchronised straight driving, auto start, Wi-Fi CSV logging with resend after a late connection | Base firmware for everything below |
| [02](02_wheel_radius/) | **Wheel radius** (Odometry Ex2) | Straight trials ±5–18 cm, least-squares fit of odometry vs ruler | r = **16.36 mm** (± 0.13) |
| [03](03_wheel_separation/) | **Wheel separation** (Odometry Ex3) | On-the-spot turns ±90…±270°, fit on ±90/±180, ±270 held back | B = **83.48 mm** (± 0.46); held-back heading error 16.6° → 4.5° |
| [04](04_square_benchmark/) | **Rectangle benchmark** (Odometry Ex4) | 321 × 217 mm loop CW and CCW, odometry vs measured closure | Residual 31 mm (CCW), 41 mm (CW), first trial each |
| [05](05_path_reproduction/) | **Path Reproduction** (group project) | Designed path (straights, arcs, rotations, reversal, speed changes) on the robot and in the Digital Twin (Processing + Python), course vs calibrated twin model | Twin calibrated from Ex2–Ex4 logs; real runs pending |

Board setup for every sketch: Arduino IDE, M5Stack board package, board **M5StampC3**, USB CDC On Boot
**Disabled**, upload speed 115200. Wi-Fi access point of the robot: `CSY`.
