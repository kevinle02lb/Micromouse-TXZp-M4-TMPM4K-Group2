# Module Testing Evidence

Verification log for the micromouse bring-up. Each module gets a test, a status,
and a piece of evidence (a serial log, a photo, or a recorded clip) proving it
works before the next layer is stacked on top.

**Status key:** &#9744; pending &nbsp;|&nbsp; &#128260; in progress &nbsp;|&nbsp; &#9989; pass &nbsp;|&nbsp; &#10060; fail

## Checklist

| # | Module | What it proves | Method | Status | Evidence |
|---|--------|----------------|--------|:------:|----------|
| 1 | UART | Serial TX works at 115200-8N1 | Print a known string, confirm in TeraTerm | &#9744; | |
| 2 | Timebase | Control tick is a true 1 kHz | Toggle a pin each tick, scope the period | &#9744; | |
| 3 | Motor | Direction + duty correct per wheel | `MOTOR_TEST` 18-phase sequence | &#9744; | |
| 4 | Encoder | Counts rise on forward, sign correct, CPS sane | Spin each wheel, read `test_enc_*` / CPS | &#9744; | |
| 5 | IR sensors | All four channels respond to a wall | `IR_TEST` stream, wave a wall past each | &#9744; | |
| 6 | Motion / PID | PV tracks SP, MV settles, no runaway | `MOTION_TEST` on a stand, plot with `plot_pid.m` | &#9744; | |
| 7 | Odometry | X / Y / heading match a known move | Drive one cell + a 90&deg; turn, compare pose | &#9744; | |
| 8 | FloodFill | Planner returns correct action for a known maze | Feed a fixed wall map, check the action sequence | &#9744; | |
| 9 | Navigator | Turn + drive one cell, heading stays gridded | Single-cell move on the floor | &#9744; | |
| 10 | Full run | Robot reaches the goal | Timed run on a real maze | &#9744; | |

## Evidence files

Drop captures next to this README (or under `../assets/` if embedded elsewhere) and
link them from the table. Suggested naming:

- `03_motor_test.mp4`, `06_pid_step_500cps.png`, `06_pid_step_500cps.txt` (raw log)
- keep the raw serial log alongside any PID plot so a run can be re-plotted later

## Notes

Free-form observations per test — what failed, what you changed, gain values tried, etc.

-