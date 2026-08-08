![status](https://img.shields.io/badge/status-in%20development-yellow)
![MCU](https://img.shields.io/badge/MCU-TMPM4KNF10AFG-blue)
![core](https://img.shields.io/badge/Arm-Cortex--M4-0091BD)
![language](https://img.shields.io/badge/language-C-A8B9CC)
![IDE](https://img.shields.io/badge/IDE-Keil%20µVision-CC0000)
![PCB](https://img.shields.io/badge/PCB-4--layer%20KiCad-663399)
![license](https://img.shields.io/badge/license-MIT-blue)

<div align="center">

# 🐭 Toshiba Micromouse - Work in Progress

**Autonomous 16×16 Maze-Solving Robot**  
*Register-level firmware on Arm Cortex-M4*

<img src="docs/assets/robot.jpg" width="520" alt="The micromouse robot">

</div>

---

## Overview

Micromouse is a classic robotics competition: an autonomous palm-sized robot explores a 16×16 grid maze, maps the walls, solves for the center, and then races back through the fastest route it discovered.

This project implements the entire stack from scratch — custom 4-layer PCB, mechanical chassis, and register-level firmware — with no reliance on external HAL libraries.

---

## Platform

<div align="center">

<img src="docs/assets/Toshiba-Logo.png" height="60" alt="Toshiba">

### **Toshiba TMPM4KNF10AFG** — Arm Cortex-M4 with FPU

</div>

The TMPM4KNF10AFG comes from the **[M4K Group](https://toshiba.semicon-storage.com/us/semiconductor/product/microcontrollers/txz4aplus-series/m4k-group.html)** of Toshiba's **[TXZ+™4A Series](https://toshiba.semicon-storage.com/us/semiconductor/product/microcontrollers/txz4aplus-series.html)** — 160 MHz Arm Cortex-M4 microcontrollers built for motor and inverter control. It pairs a fast floating-point core with dedicated on-chip hardware for driving motors, reading encoders, and sampling analog sensors, and is used across industrial and consumer motion systems such as BLDC/PMSM drives, HVAC compressors, power tools, and factory automation — with IEC 60730 self-diagnosis support for appliance functional safety.

| Capability | Role in this build |
|-----------|--------------------|
| Cortex-M4 @ 160 MHz + FPU | Navigation, PID, and flood-fill maze solving |
| On-chip motor & encoder hardware | Wheel feedback and PPG-generated motor PWM |
| High-speed analog sensing | IR wall detection |

The same silicon built for industrial three-phase drives runs a smaller-scale problem here — a 1 kHz closed-loop PID over two brushed DC gear-motors — with dedicated hardware handling encoder counting and motor PWM so the CPU stays free for navigation and flood-fill maze solving.

---

## Built With

<div align="center">

| <img src="docs/assets/arm.png" height="40" alt="ARM"> | <img src="docs/assets/Keil.png" height="40" alt="Keil µVision"> | <img src="docs/assets/kicad.png" height="40" alt="KiCad"> | <img src="docs/assets/autodesk-fusion-360_logo_.png" height="40" alt="Fusion 360"> | <img src="docs/assets/git.png" height="40" alt="Git"> | <img src="docs/assets/matlab.png" height="40" alt="MATLAB"> |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **Arm Cortex-M4** | **Keil µVision** | **KiCad** | **Fusion 360** | **Git** | **MATLAB** |
| Register-level C | IDE / Build / Flash | 4-layer PCB | Chassis CAD | Version Control | Data Analysis |

</div>

---

## Architecture Highlights

- **Flood-fill maze solver** — pure logic, zero hardware coupling
- **1 kHz per-wheel PID** speed control
- **Fully hand-written HAL** from datasheets, no external libraries
- **Relative-move navigation** — turn error never compounds across a run
- **Custom 4-layer PCB** with integrated motor drivers & IR sensors

---

## Gallery

<div align="center">

<img src="docs/assets/pcb.jpg" width="300" alt="Custom PCB"> &nbsp;&nbsp; <img src="docs/assets/maze.jpg" width="300" alt="Test maze run">

*Custom PCB and test maze run*

<br>

<img src="docs/assets/micromouse_fusion.gif" width="480" alt="Fusion 360 Robot Render">

*Fusion 360 render of the full mechanical assembly*

</div>

---

## Documentation
 
| Path | Contents |
|------|----------|
| [`src/README.md`](src/README.md) | Firmware architecture, control loops, pin map, timer config |
| [`hardware/README.md`](hardware/README.md) | Hardware overview — chassis (Fusion) + PCB (KiCad) |
| [`hardware/pcb/README.md`](hardware/pcb/README.md) | Board specs, layout, schematic, component list |
| [`docs/README.md`](docs/README.md) | Docs index — assets, evidence, and analysis tooling |
| [`docs/evidence/README.md`](docs/evidence/README.md) | Module-test checklist and captured evidence |

---

## Quick Start

```text
IDE:     Keil µVision
Target:  TMPM4KNF10AFG (Arm Cortex-M4)
Debug:   CMSIS-DAP
Flash:   512 KB on-chip
```

---

<div align="center">

**Kevin Le** &nbsp;•&nbsp; 2026 &nbsp;
</div>