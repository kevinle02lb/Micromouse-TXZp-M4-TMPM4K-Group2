#!/usr/bin/env python3
"""Log and plot micromouse telemetry straight from the UART.

The firmware prints a one-time CSV header before each test stream, so the
column layout is read from the wire rather than hardcoded here. Every row is
appended to a CSV file as it arrives and plotted live in the same pass.

Install once:
    pip install pyserial matplotlib

Examples:
    python3 tools/mmlog.py --list
    python3 tools/mmlog.py COM5
    python3 tools/mmlog.py COM5 -o turn_01.csv
    python3 tools/mmlog.py logs/turn_01.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

BAUD = 115200
WINDOW_DEFAULT = 0.0        # 0 shows the whole run, >0 shows a sliding window

# Subplot layout per firmware test mode, keyed on the header the firmware
# prints. Columns sharing units belong on one axis. A header absent from this
# table falls back to a single axis holding every non-time column.
PLOT_GROUPS = {
    "sp,pvL,pvR,mvL,mvR": (
        ("speed (CPS)", ("sp", "pvL", "pvR")),
        ("output (% duty)", ("mvL", "mvR")),
    ),
    "encL,encR,x,deg,diff": (
        ("encoder (counts)", ("encL", "encR")),
        ("odometry (mm, deg)", ("x", "deg", "diff")),
    ),
    "ms,arc,deg,pvL,pvR,sp,state": (
        ("arc (mm)", ("arc", "sp")),
        ("speed (CPS)", ("pvL", "pvR")),
        ("heading (deg), state", ("deg", "state")),
    ),
    "ms,dist,encL,encR,sp,state": (
        ("distance (mm)", ("dist", "sp")),
        ("encoder (counts)", ("encL", "encR")),
        ("state", ("state",)),
    ),
}

SATURATION_AXES = ("output (% duty)",)   # axes that get +/-100 saturation guides


def is_data_row(fields: list[str]) -> bool:
    """Report whether every field parses as a number."""
    for field in fields:
        try:
            float(field)
        except ValueError:
            return False
    return bool(fields)


class Capture:
    """Accumulates one telemetry stream and mirrors it to a CSV file.

    Two states. Before a header arrives the capture is idle and numeric rows
    are discarded, which drops the partial line left over from connecting
    mid-stream. After a header arrives the capture is active and numeric rows
    are stored. A second header resets the capture, so power-cycling the robot
    starts a clean run without restarting the tool.
    """

    def __init__(self, out_path: Path | None, live: bool = True,
                 rate: float = 0.0):
        self.header: list[str] = []
        self.rows: list[list[float]] = []
        self.t0 = time.monotonic()
        self.stamps: list[float] = []
        self.live = live
        self.rate = rate
        self._out = None
        self._writer = None
        self._path = out_path

    def _reset(self, header: list[str]) -> None:
        self.header = header
        self.rows = []
        self.stamps = []
        self.t0 = time.monotonic()

        if self._path is not None:
            if self._out is not None:
                self._out.close()
            self._out = open(self._path, "w", newline="")
            self._writer = csv.writer(self._out)
            self._writer.writerow(header)

    def feed(self, line: str) -> None:
        """Route one received line to the header or the data store."""
        fields = [f.strip() for f in line.strip().split(",")]
        if len(fields) < 2:
            return

        if not is_data_row(fields):
            self._reset(fields)
            return

        if not self.header or len(fields) != len(self.header):
            return

        self.rows.append([float(f) for f in fields])
        self.stamps.append(time.monotonic() - self.t0)

        if self._writer is not None:
            self._writer.writerow(fields)
            self._out.flush()          # a killed run still leaves a usable file

    def time_axis(self) -> tuple[list[float], str]:
        """Return the x axis values and the label describing them.

        A firmware-supplied millisecond column is authoritative. An explicit
        sample rate is next, matching the decimated stream rate the firmware
        was built with. A live capture without either falls back to host
        arrival time. A replayed file without either is plotted against the
        sample index, since arrival time carries no meaning off the wire.
        """
        if "ms" in self.header:
            index = self.header.index("ms")
            return [row[index] / 1000.0 for row in self.rows], "time (s)"

        if self.rate > 0.0:
            return [i / self.rate for i in range(len(self.rows))], "time (s)"

        if self.live:
            return self.stamps, "time (s)"

        return list(range(len(self.rows))), "sample"

    def column(self, name: str) -> list[float]:
        index = self.header.index(name)
        return [row[index] for row in self.rows]

    def close(self) -> None:
        if self._out is not None:
            self._out.close()
            self._out = None


class SerialSource:
    """Non-blocking line reader over a serial port.

    Bytes are buffered and split on newlines so a read that lands mid-line
    does not produce a truncated row.
    """

    def __init__(self, port: str, baud: int = BAUD):
        import serial
        self.port = serial.Serial(port, baud, timeout=0)
        self.buffer = b""

    def poll(self) -> list[str]:
        waiting = self.port.in_waiting
        if waiting:
            self.buffer += self.port.read(waiting)

        *complete, self.buffer = self.buffer.split(b"\n")
        return [line.decode("ascii", "replace") for line in complete]

    def close(self) -> None:
        self.port.close()


class FileSource:
    """Replays a previously saved capture in one pass."""

    def __init__(self, path: Path):
        self.lines = path.read_text(errors="replace").splitlines()

    def poll(self) -> list[str]:
        lines, self.lines = self.lines, []
        return lines

    def close(self) -> None:
        pass


def layout(header: list[str]) -> tuple:
    """Choose the subplot grouping for a header."""
    key = ",".join(header)
    if key in PLOT_GROUPS:
        return PLOT_GROUPS[key]
    return (("value", tuple(c for c in header if c != "ms")),)


def report(capture: Capture) -> None:
    """Print per-column steady-state means over the last fifth of the run."""
    if len(capture.rows) < 5:
        return

    start = max(0, int(len(capture.rows) * 0.8))
    tail = capture.rows[start:]
    print(f"\nsteady state, last {len(tail)} of {len(capture.rows)} samples")
    for i, name in enumerate(capture.header):
        values = [row[i] for row in tail]
        print(f"  {name:>6} mean {sum(values) / len(values):9.2f}")


def run(source, capture: Capture, window: float) -> None:
    """Drive the plot until the window is closed or the tool is interrupted."""
    import matplotlib.pyplot as plt

    plt.ion()
    figure = None
    axes = {}
    lines = {}

    try:
        while True:
            for line in source.poll():
                capture.feed(line)

            if capture.header and figure is None:
                groups = layout(capture.header)
                figure, created = plt.subplots(len(groups), 1, sharex=True,
                                               figsize=(10, 2.6 * len(groups)))
                created = created if len(groups) > 1 else [created]
                figure.canvas.manager.set_window_title(",".join(capture.header))

                for axis, (label, columns) in zip(created, groups):
                    axis.set_ylabel(label)
                    axis.grid(True, alpha=0.3)
                    axes[label] = axis
                    if label in SATURATION_AXES:
                        axis.axhline(100, ls=":", lw=0.8, color="0.5")
                        axis.axhline(-100, ls=":", lw=0.8, color="0.5")
                    for name in columns:
                        if name in capture.header:
                            lines[name] = axis.plot([], [], lw=1.2, label=name)[0]
                    axis.legend(loc="lower right", fontsize=8)

                created[-1].set_xlabel(capture.time_axis()[1])
                figure.tight_layout()

            if figure is not None and capture.rows:
                t = capture.time_axis()[0]
                for name, artist in lines.items():
                    artist.set_data(t, capture.column(name))
                for axis in axes.values():
                    axis.relim()
                    axis.autoscale_view()
                if window > 0 and t and t[-1] > window:
                    for axis in axes.values():
                        axis.set_xlim(t[-1] - window, t[-1])

            if isinstance(source, FileSource):
                plt.ioff()
                if figure is None:
                    print("No header row found in the file.")
                    return
                plt.show()
                return

            if figure is not None and not plt.fignum_exists(figure.number):
                return

            plt.pause(0.05)

    except KeyboardInterrupt:
        pass


def list_ports() -> None:
    from serial.tools import list_ports as tools
    found = list(tools.comports())
    if not found:
        print("No serial ports found.")
    for port in found:
        print(f"  {port.device:12} {port.description}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Log and plot micromouse UART telemetry.")
    parser.add_argument("target", nargs="?",
                        help="serial port (COM5, /dev/ttyUSB0) or a saved CSV")
    parser.add_argument("-o", "--out", help="CSV output path")
    parser.add_argument("-b", "--baud", type=int, default=BAUD)
    parser.add_argument("-w", "--window", type=float, default=WINDOW_DEFAULT,
                        metavar="SEC", help="sliding window width, 0 for all")
    parser.add_argument("--hz", type=float, default=0.0,
                        help="stream rate, used when the firmware sends no ms column")
    parser.add_argument("--list", action="store_true", help="list serial ports")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return 0

    if not args.target:
        parser.error("a serial port or CSV path is required")

    path = Path(args.target)
    if path.exists() and path.is_file():
        source, out, live = FileSource(path), None, False
        print(f"replaying {path}")
    else:
        source, live = SerialSource(args.target, args.baud), True
        out = Path(args.out) if args.out else Path(
            f"mmlog_{datetime.now():%Y%m%d_%H%M%S}.csv")
        print(f"{args.target} at {args.baud} Bd, writing {out}")
        print("waiting for a header row, Ctrl-C to stop")

    capture = Capture(out, live, args.hz)
    try:
        run(source, capture, args.window)
    finally:
        source.close()
        capture.close()

    report(capture)
    if out is not None:
        print(f"\n{len(capture.rows)} rows saved to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
