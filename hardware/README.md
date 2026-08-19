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

## Drivetrain

Two encoder gearmotors driving 80 mm wheels, held to the chassis by N20 brackets.
The encoder feeds the odometry and speed loop in firmware.

### Motors

| Spec | Detail |
|------|--------|
| Part | Pololu [#5211](https://www.pololu.com/product/5211) |
| Type | 30:1 Micro Metal Gearmotor HPCB 12V, side-connector encoder |
| Gear ratio | 29.86:1 (exact) |
| Encoder | 12 CPR quadrature on the motor shaft |
| No-load | 1100 RPM at 12 V, 80 mA |
| Stall | 0.39 kg&middot;cm, 0.75 A (theoretical) |
| Output shaft | 3 mm D, 9 mm long |
| Gearbox | 10 &times; 12 mm cross section (N20 form factor) |

Encoder cable is **not included** — 6-pin JST SH-style, ordered separately.

### Wheels

| Spec | Detail |
|------|--------|
| Part | Pololu [#3690](https://www.pololu.com/product/3690) |
| Type | Multi-Hub Wheel with inserts, black, 2-pack |
| Size | 80 &times; 10 mm, silicone tyre |
| Mounting | Collet insert &mdash; 3 mm D used here (3 mm round, 4 mm D, 4 mm round also supplied) |

The collet tightens as it compresses into the wheel disc, so it grips harder than
a press-fit wheel and is less likely to slip on the shaft.

### Brackets

Generic N20 motor brackets. Any pair sized for the 10 &times; 12 mm gearbox cross
section fits &mdash; no part number recorded, they are a commodity item.

### Firmware link

The encoder resolution at the gearbox output falls out of the two specs above:

```
12 CPR x 29.86 gear ratio = 358.3 counts per wheel revolution
```

That figure and the 80 mm wheel diameter set `COUNTS_PER_REV` and
`WHEEL_DIAMETER_MM` in [`Odometry.h`](../src/modules/Odometry.h). Both are
verified against the floor with `DRIVE_TEST` and `TURN_TEST` &mdash; see the
[test README](../src/README.md).

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