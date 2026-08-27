MATLAB PID TELEMETRY TOOLS
==========================

Two scripts read the comma separated stream emitted by MotionTest_StreamRow
in ModuleTest.c. Both expect the MOTION_TEST header:

    sp,pvL,pvR,mvL,mvR

    sp    setpoint, counts per second
    pvL   left wheel speed, counts per second
    pvR   right wheel speed, counts per second
    mvL   left PID output, percent duty, range -100 to 100
    mvR   right PID output, percent duty, range -100 to 100

Link settings, fixed by UART_Init in uart.c:

    115200 Bd, 8 data bits, no parity, 1 stop bit, no flow control
    line terminator CR then LF


FILES
-----

capture_pid.m   Reads the UART directly, writes a CSV, plots while capturing.
plot_pid.m      Reads a saved file and plots it. No serial port involved.


CHOOSING BETWEEN THEM
---------------------

capture_pid.m   Use during tuning. One step from robot to plot.
plot_pid.m      Use to review a run already saved, or a TeraTerm capture.

A COM port allows one program at a time. TeraTerm and capture_pid.m cannot
both hold it. Close TeraTerm before running capture_pid.m.


CAPTURE_PID.M
-------------

Settings at the top of the file:

    line  7   port          serial port name
    line  8   duration_s    capture length in seconds
    line  9   print_every   must match MOTION_TEST_PRINT_EVERY in firmware
    line 10   loop_hz       control loop rate, 1000

Steps:

    1.  Put blocks under the robot. Wheels off the ground. MotionTest_Run
        commands 250 CPS before entering its loop, so the wheels turn as
        soon as the board boots.
    2.  Flash the MOTION_TEST build in Keil.
    3.  Plug in the USB serial cable.
    4.  Close TeraTerm if it is open.
    5.  Open capture_pid.m in MATLAB.
    6.  In the Command Window, run:  serialportlist
    7.  Set port on line 7 to the port from step 6. Save.
    8.  Press Run.
    9.  Wait for the Command Window to print:
            Waiting for the header row, press RESET on the robot.
    10. Press RESET on the robot.
    11. Wait for the capture to finish. The Command Window prints the row
        count and the CSV filename.

Step 10 must come after step 9. The firmware prints the header once at
boot, so MATLAB has to be listening before the board starts.

Output file:

    pidlog_YYYYMMDD_HHMMSS.csv

Written row by row during the capture, so an interrupted run still leaves a
readable file. The format is the input format plot_pid.m reads.


PLOT_PID.M
----------

Settings at the top of the file:

    line  5   file          path to the log
    line  6   print_every   must match MOTION_TEST_PRINT_EVERY in firmware
    line  7   loop_hz       control loop rate, 1000

Steps:

    1.  Set file on line 5 to the log to plot. This accepts a CSV from
        capture_pid.m or a TeraTerm text capture.
    2.  Press Run.

Capturing with TeraTerm instead of capture_pid.m:

    File > Log, "Plain text" on, "Timestamp" off.

Timestamps left on prepend text to every line, which makes the rows
non numeric and produces an empty plot.


OUTPUT COMMON TO BOTH
---------------------

Upper axis   setpoint against left and right wheel speed
Lower axis   left and right controller output, with saturation at +/-100
             marked by dotted lines

The two axes share their time axis, so zooming one zooms the other.

The Command Window prints steady state figures over the last 20 percent of
the run: mean setpoint, mean speed and error per side, mean output per side,
and peak overshoot per side.


READING THE RESULT
------------------

Steady state error non zero and equal on both sides
    Integral gain too low, or the deadzone compensation is short.

One side settles at a different speed than the other
    Mechanical or per wheel calibration difference, not a gain problem.

Output pinned at +100 or -100 for long stretches
    The controller is saturated. Gains beyond that point have no effect.

Overshoot with visible ringing
    Proportional gain too high for the current derivative gain.


TROUBLESHOOTING
---------------

Port COM5 is unavailable
    Another program holds the port. Close TeraTerm.

No data on COM5
    Wrong port, cable not connected, or RESET was not pressed. Repeat from
    step 6.

Script sits at "Waiting for the header row" and never proceeds
    The board was already streaming before MATLAB opened the port, so the
    header had already gone out. Press Ctrl-C in the Command Window, then
    repeat from step 8.

Header is "...", expected "sp,pvL,pvR,mvL,mvR"
    A different test build is flashed. Rebuild with MOTION_TEST defined.

Plot appears but the time axis is wrong
    print_every does not match MOTION_TEST_PRINT_EVERY in ModuleTest.c.

MATLAB reports the port is in use after an error
    capture_pid.m releases the port through onCleanup, including on Ctrl-C.
    If a port is still held after a crash, run:  clear all
