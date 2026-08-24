#!/usr/bin/env python3
"""Exercise the firmware flood fill planner on the host.

Loads the compiled planner through ctypes and drives it against generated
mazes, so the code under test is the same object code that runs on the robot
rather than a Python reimplementation.

Build the library first:
    cmake -S tools -B build && cmake --build build

Examples:
    python3 tools/maze_sim.py --seed 7 --render
    python3 tools/maze_sim.py --sweep 500
"""

from __future__ import annotations

import argparse
import ctypes
import random
import re
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


def find_header() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here / "../src/modules/FloodFill.h",
                      here / "../src/FloodFill.h",
                      here / "_stage/FloodFill.h"):
        if candidate.exists():
            return candidate.resolve()
    sys.exit("FloodFill.h not found; checked ../src/modules, ../src, tools/_stage")


ACTION = parse_enum(find_header(), "floodfill_t")
STOP = ACTION["FLOODFILL_STOP"]
FORWARD = ACTION["FLOODFILL_FORWARD"]
TURN_LEFT = ACTION["FLOODFILL_TURN_LEFT"]
TURN_RIGHT = ACTION["FLOODFILL_TURN_RIGHT"]
TURN_AROUND = ACTION["FLOODFILL_TURN_AROUND"]


def load_planner() -> ctypes.CDLL:
    """Locate and load the host build of the planner."""
    names = ("libmicromouse_sim.so", "libmicromouse_sim.dylib", "micromouse_sim.dll")
    here = Path(__file__).resolve().parent
    roots = (Path("build"), here.parent / "build", here / "build")

    for root in roots:
        for name in names:
            candidate = root / name
            if candidate.exists():
                lib = ctypes.CDLL(str(candidate.resolve()))
                break
        else:
            continue
        break
    else:
        sys.exit("libmicromouse_sim not found. "
                 "Run: cmake -S tools -B build && cmake --build build")

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
    args = parser.parse_args()

    lib = load_planner()

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
