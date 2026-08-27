#!/usr/bin/env python3
"""Exercise the firmware flood fill planner on the host.

Loads the compiled planner through ctypes and drives it against generated
mazes, so the code under test is the same object code that runs on the robot
rather than a Python reimplementation.

FloodFill.c is compiled on demand by a direct compiler call, so the only
requirement is gcc, clang, or MinGW-w64 on PATH.

Examples:
    python3 tools/maze_sim.py --seed 7 --render
    python3 tools/maze_sim.py --sweep 500
"""

from __future__ import annotations

import argparse
import ctypes
import platform
import random
import re
import shutil
import struct
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

MAZE_SIZE = 16
STEP_LIMIT = 400

# Clockwise from north, matching floodfill_dir_t.
NORTH, EAST, SOUTH, WEST = range(4)
WALL_BIT = (0x01, 0x02, 0x04, 0x08)
OPPOSITE = (SOUTH, WEST, NORTH, EAST)
DELTA = ((0, 1), (1, 0), (0, -1), (-1, 0))

GOAL_CELLS = {(7, 7), (7, 8), (8, 7), (8, 8)}


def parse_enum(header: Path, name: str) -> dict:
    """Read enumerator values straight out of a C header.

    The action codes live in FloodFill.h. Copying them into this file would
    let the two drift apart silently, so they are parsed instead.
    """
    text = header.read_text()
    match = re.search(r"typedef\s+enum\s*\{([^}]*)\}\s*" + name + r"\s*;", text)
    if not match:
        sys.exit(f"{name} not found in {header}")

    # Strip comments before splitting: Doxygen text contains commas.
    body = re.sub(r"/\*.*?\*/", "", match.group(1), flags=re.S)

    values = {}
    nxt = 0
    for line in body.split(","):
        line = line.strip()
        if not line:
            continue
        if "=" in line:
            label, literal = (part.strip() for part in line.split("=", 1))
            nxt = int(literal.rstrip("uUlL"), 0)
        else:
            label = line
        values[label] = nxt
        nxt += 1
    return values


def find_sources() -> tuple[Path, Path]:
    """Locate FloodFill.h and FloodFill.c relative to this script.

    The script directory and its first three parents are searched, each with a
    short list of common source subdirectories, so the tool runs unchanged
    whether it sits beside the firmware or in a tools subdirectory.
    """
    here = Path(__file__).resolve().parent
    roots = [here, *list(here.parents)[:3]]
    subdirs = ("", "src", "src/modules", "Core/Src", "firmware")

    for root in roots:
        for sub in subdirs:
            base = root / sub if sub else root
            header, source = base / "FloodFill.h", base / "FloodFill.c"
            if header.exists() and source.exists():
                return header.resolve(), source.resolve()

    sys.exit(f"FloodFill.h and FloodFill.c not found at or above {here}")


HEADER, SOURCE = find_sources()
ACTION = parse_enum(HEADER, "floodfill_t")
STOP = ACTION["FLOODFILL_STOP"]
FORWARD = ACTION["FLOODFILL_FORWARD"]
TURN_LEFT = ACTION["FLOODFILL_TURN_LEFT"]
TURN_RIGHT = ACTION["FLOODFILL_TURN_RIGHT"]
TURN_AROUND = ACTION["FLOODFILL_TURN_AROUND"]


def compiler_target(compiler: str) -> str:
    """Return the target triple a compiler emits code for, empty if unknown."""
    try:
        done = subprocess.run([compiler, "-dumpmachine"],
                              capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.SubprocessError):
        return ""
    return done.stdout.strip() if done.returncode == 0 else ""


def host_tokens() -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Return the architecture and platform tokens a host triple must contain.

    Bitness comes from the running interpreter rather than the operating
    system, because a 32-bit interpreter cannot load a 64-bit library even on
    a 64-bit machine.
    """
    machine = platform.machine().lower()
    sixty_four = struct.calcsize("P") == 8

    if machine.startswith(("arm", "aarch")):
        arch = ("aarch64", "arm64") if sixty_four else ("arm",)
    elif sixty_four:
        arch = ("x86_64", "amd64")
    else:
        arch = ("i386", "i486", "i586", "i686")

    system = {"win32": ("mingw", "windows", "cygwin", "msvc"),
              "darwin": ("darwin", "apple")}.get(sys.platform, ("linux", "gnu"))
    return arch, system


def find_compiler(preferred: str | None = None) -> tuple[str, str]:
    """Select a compiler that targets this host.

    A cross compiler builds without error and produces a library the loader
    then rejects, so the target triple is checked here rather than leaving the
    failure to ctypes. Every candidate found is reported when none qualifies.
    """
    arch, system = host_tokens()
    names = [preferred] if preferred else [
        "cc", "gcc", "clang", "x86_64-w64-mingw32-gcc", "i686-w64-mingw32-gcc"]

    seen = []
    for name in names:
        path = shutil.which(name)
        if path is None:
            continue
        triple = compiler_target(path).lower()
        seen.append(f"    {path}\n        targets {triple or 'unknown'}")
        if any(a in triple for a in arch) and any(s in triple for s in system):
            return path, triple

    if not seen:
        sys.exit("No C compiler on PATH. Install MinGW-w64, gcc, or clang, "
                 "or pass --cc with a full path.")

    sys.exit("No compiler on PATH targets this host.\n"
             f"  need a triple containing one of {arch} and one of {system}\n"
             "  found:\n" + "\n".join(seen) +
             "\n  pass --cc with the path to a host compiler.")


def build_planner(force: bool = False, cc: str | None = None) -> Path:
    """Compile the planner into a shared library beside this script.

    FloodFill.c is one translation unit that includes only stdint.h and
    stdbool.h, so a single compiler call is enough and no build system is
    involved. The library is rebuilt only when the source is newer than it.
    """
    suffix = {"win32": ".dll", "darwin": ".dylib"}.get(sys.platform, ".so")
    out = Path(__file__).resolve().parent / ("libmicromouse_sim" + suffix)

    fresh = out.exists() and out.stat().st_mtime >= SOURCE.stat().st_mtime
    if fresh and not force and cc is None:
        return out

    compiler, triple = find_compiler(cc)
    cmd = [compiler, "-O2", "-std=c11", "-Wall", "-shared"]
    if sys.platform != "win32":
        cmd.append("-fPIC")
    cmd += [str(SOURCE), "-o", str(out)]

    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode != 0:
        sys.exit(f"Build failed using {compiler} ({triple}):\n{done.stderr}")
    if done.stderr:
        print(done.stderr, file=sys.stderr)

    print(f"built {out.name} with {compiler} ({triple})")
    return out


def load_planner(force: bool = False, cc: str | None = None) -> ctypes.CDLL:
    """Build the planner if needed, then load it and declare its signatures."""
    library = build_planner(force, cc)
    try:
        lib = ctypes.CDLL(str(library))
    except OSError as error:
        sys.exit(f"{library.name} did not load: {error}\n"
                 "  The library does not match this interpreter. Rebuild with "
                 "a host compiler:\n"
                 "    python maze_sim.py --rebuild --cc <path to gcc>")

    lib.FloodFill_Init.restype = None
    lib.FloodFill_SetWalls.argtypes = (ctypes.c_bool,) * 3
    lib.FloodFill_SetWalls.restype = None
    lib.FloodFill_Plan.restype = ctypes.c_int
    lib.FloodFill_ReportDone.argtypes = (ctypes.c_int,)
    lib.FloodFill_ReportDone.restype = None
    lib.FloodFill_GetX.restype = ctypes.c_uint8
    lib.FloodFill_GetY.restype = ctypes.c_uint8
    lib.FloodFill_IsAtGoal.restype = ctypes.c_bool
    return lib


class Maze:
    """Ground truth the simulated robot senses against.

    Generated as a spanning tree so every cell is reachable, then opened up
    with extra passages to produce the loops a competition maze contains. The
    planner never sees this structure; it learns walls only through the three
    readings passed to FloodFill_SetWalls, exactly as on the robot.
    """

    def __init__(self, seed: int, loops: float = 0.15):
        if seed == 0:
            self.cells = [[0] * MAZE_SIZE for _ in range(MAZE_SIZE)]
            self._seal_perimeter()
            return

        # Every cell starts fully enclosed, then passages are carved out.
        self.cells = [[0x0F] * MAZE_SIZE for _ in range(MAZE_SIZE)]
        rng = random.Random(seed)
        self._carve_spanning_tree(rng)
        self._add_loops(rng, loops)
        self._seal_perimeter()

    def _seal_perimeter(self) -> None:
        for i in range(MAZE_SIZE):
            self.cells[i][0] |= WALL_BIT[SOUTH]
            self.cells[i][MAZE_SIZE - 1] |= WALL_BIT[NORTH]
            self.cells[0][i] |= WALL_BIT[WEST]
            self.cells[MAZE_SIZE - 1][i] |= WALL_BIT[EAST]

    def _open(self, x: int, y: int, direction: int) -> None:
        """Remove the wall between a cell and its neighbour, on both sides."""
        dx, dy = DELTA[direction]
        nx, ny = x + dx, y + dy
        if not (0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE):
            return
        self.cells[x][y] &= ~WALL_BIT[direction]
        self.cells[nx][ny] &= ~WALL_BIT[OPPOSITE[direction]]

    def _carve_spanning_tree(self, rng: random.Random) -> None:
        """Randomised depth-first carve, which reaches every cell exactly once."""
        seen = {(0, 0)}
        stack = [(0, 0)]

        while stack:
            x, y = stack[-1]
            options = []
            for direction in range(4):
                dx, dy = DELTA[direction]
                nx, ny = x + dx, y + dy
                if 0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE and (nx, ny) not in seen:
                    options.append((direction, nx, ny))

            if not options:
                stack.pop()
                continue

            direction, nx, ny = rng.choice(options)
            self._open(x, y, direction)
            seen.add((nx, ny))
            stack.append((nx, ny))

    def _add_loops(self, rng: random.Random, fraction: float) -> None:
        """Open extra passages so the maze is not a pure tree."""
        for _ in range(int(MAZE_SIZE * MAZE_SIZE * fraction)):
            self._open(rng.randrange(MAZE_SIZE), rng.randrange(MAZE_SIZE),
                       rng.randrange(4))

    def blocked(self, x: int, y: int, direction: int) -> bool:
        return bool(self.cells[x][y] & WALL_BIT[direction])


@dataclass
class Result:
    seed: int
    outcome: str                      # reached, stalled, or escaped
    steps: int
    path: list = field(default_factory=list)


def run(lib: ctypes.CDLL, maze: Maze, seed: int) -> Result:
    """Drive the planner to the goal, the step limit, or an invalid position."""
    lib.FloodFill_Init()
    heading = NORTH
    path = []

    for step in range(STEP_LIMIT):
        x, y = int(lib.FloodFill_GetX()), int(lib.FloodFill_GetY())

        # The planner tracks its own believed cell. If that leaves the maze the
        # firmware would index outside walls[][], so stop and flag it.
        if not (0 <= x < MAZE_SIZE and 0 <= y < MAZE_SIZE):
            return Result(seed, "escaped", step, path)

        path.append((x, y))

        lib.FloodFill_SetWalls(
            maze.blocked(x, y, heading),
            maze.blocked(x, y, (heading + 3) & 3),
            maze.blocked(x, y, (heading + 1) & 3),
        )

        action = lib.FloodFill_Plan()
        if action == STOP:
            return Result(seed, "reached", step, path)

        if action == TURN_LEFT:
            heading = (heading + 3) & 3
        elif action == TURN_RIGHT:
            heading = (heading + 1) & 3
        elif action == TURN_AROUND:
            heading = (heading + 2) & 3

        lib.FloodFill_ReportDone(action)

    return Result(seed, "stalled", STEP_LIMIT, path)


def render(maze: Maze, path: list) -> str:
    """Draw the maze with the travelled path marked."""
    visited = set(path)
    lines = []
    for y in range(MAZE_SIZE - 1, -1, -1):
        top = ""
        mid = ""
        for x in range(MAZE_SIZE):
            top += "+---" if maze.blocked(x, y, NORTH) else "+   "
            mid += "|" if maze.blocked(x, y, WEST) else " "
            if (x, y) in GOAL_CELLS:
                mid += " G "
            elif (x, y) == (0, 0):
                mid += " S "
            elif (x, y) in visited:
                mid += " . "
            else:
                mid += "   "
        lines.append(top + "+")
        lines.append(mid + "|")
    lines.append("+---" * MAZE_SIZE + "+")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the firmware flood fill planner against generated mazes.")
    parser.add_argument("--seed", type=int, default=1,
                        help="maze seed, 0 for an open maze")
    parser.add_argument("--loops", type=float, default=0.15,
                        help="fraction of extra passages beyond a spanning tree")
    parser.add_argument("--render", action="store_true",
                        help="draw the maze and path")
    parser.add_argument("--sweep", type=int, metavar="N",
                        help="run seeds 1..N and summarise")
    parser.add_argument("--rebuild", action="store_true",
                        help="force a recompile of the planner")
    parser.add_argument("--cc", metavar="PATH",
                        help="C compiler to build the planner with")
    args = parser.parse_args()

    lib = load_planner(args.rebuild, args.cc)

    if args.sweep:
        reached = 0
        steps = 0
        bad = {"stalled": [], "escaped": []}

        for seed in range(1, args.sweep + 1):
            result = run(lib, Maze(seed, args.loops), seed)
            if result.outcome == "reached":
                reached += 1
                steps += result.steps
            else:
                bad[result.outcome].append(seed)

        print(f"{args.sweep} mazes")
        mean = f", mean {steps / reached:.1f} steps" if reached else ""
        print(f"  reached  {reached}{mean}")
        for key in ("stalled", "escaped"):
            if bad[key]:
                print(f"  {key:8} {len(bad[key]):4}  seeds {bad[key][:15]}")
        return 1 if (bad["stalled"] or bad["escaped"]) else 0

    maze = Maze(args.seed, args.loops)
    result = run(lib, maze, args.seed)

    print(f"seed {args.seed}: {result.outcome} in {result.steps} steps, "
          f"{len(set(result.path))} cells visited")

    if args.render:
        print()
        print(render(maze, result.path))

    return 0 if result.outcome == "reached" else 1


if __name__ == "__main__":
    sys.exit(main())