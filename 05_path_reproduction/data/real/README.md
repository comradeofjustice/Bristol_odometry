Put the real robot logs here:

- `path_t<N>.csv` — files saved by `Path_Reproduction/wifi_logger.py` (one per run).
- `checkpoints.csv` (optional) — physical positions measured on the sheet at segment ends, from a run with
  `CHECKPOINT_PAUSE_MS = 3000`. Columns: `trial,segment,x_mm,y_mm,heading_deg` in the start frame
  (origin = start position, x = start heading, y = robot's left, heading anticlockwise positive).
