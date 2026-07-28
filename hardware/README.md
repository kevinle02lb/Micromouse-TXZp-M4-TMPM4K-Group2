<div align="center">

# Hardware

**Physical design layer — mechanical + electrical**  
*3D-printed chassis in Fusion, custom control PCB in KiCad*

</div>

---

## Overview

The `hardware/` directory holds everything physical about the micromouse. The mechanical design — chassis and overall body — lives in Fusion, while the electronics — the custom 4-layer control board — live in KiCad. Each has its own subfolder with its own design files.

---

## Structure

| Folder | Tool | Contents |
|--------|------|----------|
| [`fusion/`](fusion/) | Fusion | Chassis and mechanical model — STL for printing, STEP for CAD |
| [`pcb/`](pcb/) | KiCad | 4-layer control board — schematic, layout, gerbers, BOM |

---

## Mechanical — Fusion

The chassis is designed in Fusion and 3D-printed. It carries the PCB, motors, and wheels, and defines the overall footprint of the mouse.

<!-- Add a render here once you have one, e.g.:
<div align="center">
<img src="../docs/assets/chassis-3d.png" width="480" alt="Chassis render">
</div>
-->

| Spec | Detail |
|------|--------|
| Process | MJF (Multi Jet Fusion), via JLC3DP |
| Material | PA12-HP nylon |
| Finish | Dyed black (Optional) |
| Dimensions | 10.8 × 8.24 × 5.74 cm |
| Volume | 17.39 cm³ |

### Files

| File | Contents |
|------|----------|
| [`fusion/chassis.stl`](fusion/chassis.stl) | Print-ready chassis mesh (STL) |
| [`fusion/Micromouse_v1.step`](fusion/Micromouse_v1.step) | Parametric mechanical model (STEP) |

---

## Electrical — PCB

A custom 4-layer control board designed in KiCad, carrying the MCU, dual motor drivers, IR wall sensors, and the power and debug interface on a single micromouse-sized board.

| Spec | Detail |
|------|--------|
| Layers | 4-layer |
| MCU | Toshiba TMPM4KNF10AFG (Cortex-M4F) |
| Motor drivers | 2 × TB67H450AFNG H-bridge |
| Sensing | 4 × IR emitter / receiver pairs |
| Supply | 5 V |

Full details, schematic, and design files are in the PCB README:

**→ [`pcb/README.md`](pcb/README.md)**

---

<div align="center">

[← Back to main README](../README.md)

</div>