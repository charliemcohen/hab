# Reaction-wheel telemetry format

This is what `HAB_RW_Controller.ino` (version `HAB-RW 1.2.0-ground`) prints over its serial port. Nothing recorded this output during Flight 2, so there's no file in this format in the repo. I'm documenting it for bench testing and for the next flight, when I want to log it to SD.

The controller prints one line of 36 comma-separated numbers about every 100 ms (10 Hz). It also prints lines that start with `#`, which are a header (`#HDR`), events, and status messages.

## Fields, in order

| # | Field | Meaning |
|---:|---|---|
| 1 | `t_us` | microseconds since the Nano booted |
| 2 | `state` | controller state code |
| 3 | `tgt_cdeg` | target heading, 0.01° |
| 4 | `psi_cdeg` | measured heading, 0.01° |
| 5 | `err_cdeg` | heading error, 0.01° |
| 6 | `r_mdps` | yaw rate, 0.001°/s |
| 7 | `rcmd_mdps` | commanded yaw rate, 0.001°/s |
| 8 | `w_drpm` | wheel speed, RPM × 10 |
| 9 | `wref_drpm` | target wheel speed, RPM × 10 |
| 10 | `walpha_rpmps` | estimated wheel acceleration, RPM/s |
| 11 | `enc` | encoder count |
| 12 | `inv` | invalid encoder transitions |
| 13 | `taucmd_uNm` | torque estimate, µN·m |
| 14 | `alphacmd_rpmps` | commanded wheel acceleration, RPM/s |
| 15 | `u_pwm` | motor PWM actually applied |
| 16 | `mode` | motor mode code |
| 17 | `authp_mdps` | how much positive yaw rate the wheel can still correct, 0.001°/s |
| 18 | `authn_mdps` | how much negative yaw rate the wheel can still correct, 0.001°/s |
| 19 | `H_uNms` | wheel angular momentum estimate, µN·m·s |
| 20 | `gyro_age_ms` | time since the last gyro reading, ms |
| 21 | `att_age_ms` | time since the last orientation reading, ms |
| 22 | `loopmax_us` | longest control loop, µs |
| 23 | `faults_hex` | fault flags (hex) |
| 24 | `flags_hex` | status flags (hex) |
| 25 | `tilt_cdeg` | tilt, 0.01° |
| 26 | `trim_cpwm` | PWM trim, 0.01 PWM units |
| 27 | `stack_free` | estimated free stack, bytes |
| 28 | `miss` | missed loop deadlines |
| 29 | `gyro_gaps` | gaps in gyro reports |
| 30 | `grv_gaps` | gaps in rotation-vector reports |
| 31 | `vbat_mv` | battery voltage in mV, or -1 if disabled |
| 32 | `maxint_us` | longest timing interval, µs |
| 33 | `i2c_err` | I²C errors |
| 34 | `imu_bad` | bad IMU reports |
| 35 | `first_fault` | first fault ID, or -1 if none |
| 36 | `pol_changes` | motor polarity changes |

Fields 23 and 24 print as hex without a `0x` prefix. Everything else is a decimal integer.

This is not the format of `SCI_000.CSV`. That file is documented in the main README.
