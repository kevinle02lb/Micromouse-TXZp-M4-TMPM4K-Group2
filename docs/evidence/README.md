# Module Testing

Each test is a `#define` at the top of `ModuleTest.c`. Uncomment one, build, run.
One at a time. Work down the table — later tests assume earlier ones pass.

**Status:** &#9744; pending &nbsp;|&nbsp; &#128260; in progress &nbsp;|&nbsp; &#9989; pass &nbsp;|&nbsp; &#10060; fail

| # | Define | Proves | Pass criteria | Status | Evidence |
|---|--------|--------|---------------|:------:|----------|
| 1 | `IR_EMITTER_TEST` | Emitters drive | All four light in the counter pattern | &#9989;  | "evidence\01_IR_EMITTER_TEST.mp4" |
| 2 | `MOTOR_TEST` | Direction and duty per wheel | 18 phases match the table comment; `test_enc_left` rises on FORWARD, falls on REVERSE | &#9989; | "evidence\02_MOTOR_TEST.mp4" |
| 3 | `IR_TEST` | Channels respond to a wall | `FL L R FR` rise as a wall nears, fall when removed | &#9989; | "evidence\03_IR_TEST.mp4" |
| 4 | `MOTION_TEST` | Speed loop tracks a setpoint | `pvL`/`pvR` converge on `sp` without saturating | &#9989; | "evidence\04_MOTION_500CPS.mp4", "evidence\04_MOTION_1000CPS.pdf, "evidence\04_MOTION_1000CPS.txt"|
| 5 | `ODOM_TEST` | Pose maths, encoder signs | Pushed straight: `encL ≈ encR`, `deg ≈ 0`. Rotated CCW: `deg` rises, `diff` positive | &#9989; | "evidence\05_ODOM_TEST.mp4" |
| 6 | `DRIVE_TEST` | Distance calibrated | Final `dist` matches a ruler within 2 mm | &#9989; | "evidence\06_DRIVE.mp4" , "evidence\06_DRIVE.txt"|
| 7 | `TURN_TEST` | Angle calibrated | Final `arc` reaches `target`; protractor reads 90&deg; | &#9989; | "evidence\07_TURN_TEST.mp4" , "evidence\07_TURN_DATA.txt" |
| 8 | Full run (`main.c`) | Robot solves the maze | Reaches the 2&times;2 goal | &#128260; | |

## Watch out

`deg` is computed from `arc` by a fixed constant, and `dist` from the encoder
count. Neither can disagree with itself. **A ruler or protractor is the only
real check on tests 6 and 7.**

In the last rows of either, `sp` should taper toward zero. If it stops at the
speed floor instead, the wheels are still moving when the segment ends.

## Evidence

Photo or clip is fine — no write-up needed. Drop it in `evidence/` and link it
from the table.

```
evidence/02_motor_phases.mp4
evidence/06_drive_180mm.jpg     ruler shot
evidence/07_turn_90deg.jpg      protractor shot
```

Keep the raw serial log next to any plot so a run can be re-plotted.

## Calibration

Values live in the code, not here. Re-run the test if the hardware changes.

- `WHEEL_DIAMETER_MM`, `COUNTS_PER_REV` (`Odometry.h`) &rarr; `DRIVE_TEST`
- `WHEELBASE_MM` (`Odometry.h`) &rarr; `TURN_TEST`
- `ir_cal[][]`, `IR_SIDE_TARGET_MM`, `IR_FRONT_TARGET_MM` &rarr; `IR_TEST`

## Notes

-