# Test data

Raw data behind the AME 101 Monarch of the Hill group report (Team 2). Only data the report uses is here.

| File | What it holds | Used in the report |
| --- | --- | --- |
| `pitch-runs.csv` | Dashboard run log of the three autonomous climbs used for the pitch test | Section 7: the pitch figure (Fig. 11) and the pitch-error table |
| `robot-measurements.csv` | Measured mass and footprint of the finished robot | Mass in Section 3 and throughout |

## pitch-runs.csv

Logged over Bluetooth by the dashboard (`photon/ble_robot` firmware) at about 10 rows per second, and exported from its Run log tab on Oct. 23, 2026. Each row is one telemetry sample. The report's Runs A, B and C are runs 3, 5 and 2 in this file. The other runs from that session were not used and are left out.

| Column | Meaning |
| --- | --- |
| `run`, `label` | Run number and the note typed in the dashboard before the run |
| `t_s` | Time since the recording started, in seconds |
| `stage` | Autopilot state: Idle, Armed, Climb, Brake or Hold |
| `stage_s`, `run_s` | Time in the current stage and since the run started, in seconds |
| `pitch_fused_deg` | Pitch from the complementary filter, the estimate the autopilot uses, in degrees |
| `pitch_accel_deg` | Pitch from the accelerometer alone, in degrees |
| `roll_deg`, `yaw_deg` | Roll and yaw estimates, in degrees |
| `accel_trusted` | 1 when the filter was blending in the accelerometer on that sample, 0 when it followed the gyro alone |
| `motors_driven` | 1 while the motors were commanded |
| `auto_left`, `auto_right` | Autopilot motor commands, left and right |
| `accel_x_g`, `accel_y_g`, `accel_z_g` | Raw accelerometer readings, in g, in the sensor's axes |
| `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps` | Gyro readings with the offset removed, in degrees per second, in the sensor's axes |
| `robot_ms`, `motor_a`, `motor_b`, `pack_v`, `pack_raw`, `true_pitch_deg`, `mark` | Not recorded in these runs (empty) |

The true pitch is not in the file: as Section 7 explains, it is the hill's geometry, 33.4 degrees on the ramp and 0 on the mesa. The plotted window of each run is also in `report/figures/src/fig11.json`, which `fig11_pitch.py` turns into Fig. 11.

## robot-measurements.csv

The finished robot's mass and footprint, measured by the team and copied from our shared "Monarch of the Hill Data" sheet.
