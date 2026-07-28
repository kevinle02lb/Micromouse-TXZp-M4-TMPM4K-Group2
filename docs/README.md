# Documentation

Reference material, test evidence, and analysis tooling for the Toshiba micromouse.

## Contents

| Folder | Contents |
|--------|----------|
| [`assets/`](assets/) | Images used across the docs — diagrams, photos, screenshots. |
| [`evidence/`](evidence/) | Module-testing evidence: a per-module checklist plus captured serial logs and videos. |
| [`matlab/`](matlab/) | Analysis scripts. `plot_pid.m` plots UART telemetry (setpoint vs. actual speed and controller output) for PID tuning. |

## Conventions

- Images referenced by any README live in `assets/` and are linked with a relative path.
- Test recordings and captured serial logs go in `evidence/`, one per module test.
- Analysis scripts read the comma-separated telemetry streamed by the firmware test harness (`ModuleTest.c`).