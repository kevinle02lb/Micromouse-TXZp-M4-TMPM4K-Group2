# Command-Line Build Setup

Builds the firmware with `arm-none-eabi-gcc` and flashes over CMSIS-DAP,
without uVision. The Keil project keeps working the whole time; nothing here
modifies it.

Work through the parts in order. Each ends with a checkpoint. If a checkpoint
fails, stop there rather than continuing.

---

## Part 1: File placement

Put the four new files at the repository root, next to `src/`:

```
toshibaMicromouse/
  CMakeLists.txt              <- new, firmware build
  startup_TMPM4KNA.S          <- new, capital .S
  TMPM4KNF10A.ld              <- new
  cmake/
    arm-none-eabi.cmake       <- new
  src/
    main.c
    ModuleTest.c
    modules/                  Encoder.c FloodFill.c IrSensor.c Motion.c ...
    drivers/                  adc.c gpio.c timer32A.c uart.c ...
  tools/                      host-side Python, unrelated to the firmware
```

The extension on `startup_TMPM4KNA.S` must be a capital `S`. Lowercase `.s`
skips the C preprocessor, and CMake treats the two differently.

Leave the existing `startup_TMPM4KNA.s` and `.scat` alone. Keil still needs
them.

### Find the device header

`adc.h` includes `TMPM4KyA.h`. Locate that file now, because the build needs
its directory. In uVision: Project → Options for Target → C/C++ → Include
Paths. Copy whichever path contains it.

It normally lives inside Toshiba's Device Family Pack, under a path like
`C:/Keil_v5/ARM/PACK/Toshiba/...`.

### Point CMakeLists at your tree

Open `CMakeLists.txt` and check two things:

1. Every file under `add_executable` matches a real path. Fix any that do not.
2. The `cmsis` entry in `target_include_directories` points at the directory
   holding `TMPM4KyA.h`.

**Checkpoint:** the four new files exist, and you know the device header path.

---

## Part 2: Tools

Four installs. All free, all command line.

| Tool | Purpose | Source |
|---|---|---|
| Arm GNU Toolchain | `arm-none-eabi-gcc` | developer.arm.com downloads |
| CMake | generates the build | cmake.org, or `winget install Kitware.CMake` |
| Ninja | runs the build | ninja-build.org, or `winget install Ninja-build.Ninja` |
| pyOCD | flashes over CMSIS-DAP | `pip install pyocd` |

During the Arm toolchain installer, tick the option to add it to PATH. If you
miss it, pass `-DARM_TOOLCHAIN_DIR=C:/path/to/bin` when configuring.

Verify all four:

```
arm-none-eabi-gcc --version
cmake --version
ninja --version
pyocd --version
```

**Checkpoint:** four version numbers, no "not recognised".

---

## Part 3: First build

From the repository root:

```
cmake -S . -B build-fw -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build-fw
```

The first command generates the build; the second compiles. Only rerun the
first if you add or remove a source file.

Expect a memory report at the end:

```
Memory region         Used Size  Region Size  %age Used
           FLASH:       xxxxx B         1 MB      x.xx%
         RAMFUNC:           0 B         1 KB      0.00%
             RAM:        xxxx B        63 KB      x.xx%
```

Compare FLASH against the code size uVision reports. They will not match
exactly, since the compilers differ, but the same order of magnitude means
the build is sane. A FLASH figure near zero means most of your code was
dropped, usually a missing source file.

`build-fw/micromouse.hex` and `.bin` are now on disk.

**Checkpoint:** a memory report and a `.hex` file.

---

## Part 4: Flashing

Plug the CMSIS-DAP probe in and confirm it is visible:

```
pyocd list
```

If nothing appears, stop. Nothing downstream works until the probe is seen.

pyOCD needs the flash algorithm from Toshiba's CMSIS pack. Try the index:

```
pyocd pack install TMPM4KNF10A
pyocd list --targets
```

If the index does not have it, use the pack uVision already downloaded. Search
`C:/Keil_v5/ARM/PACK/Toshiba/` for a `.pack` file and pass the path below.
It is the same file, containing the same algorithm.

Reconfigure with the target name:

```
cmake -S . -B build-fw -G Ninja ^
      -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake ^
      -DPYOCD_TARGET=<name from pyocd list --targets> ^
      -DPYOCD_PACK=<path to .pack, only if the index lacked it>

cmake --build build-fw --target flash
```

**Checkpoint:** the board runs the same as the Keil build. Same UART output,
same behaviour.

---

## Part 5: Debugging

```
cmake --build build-fw --target gdbserver
```

Then in a second terminal:

```
arm-none-eabi-gdb build-fw/micromouse.elf
target remote :3333
monitor reset halt
break main
continue
```

VS Code's cortex-debug extension attaches to the same port 3333 and gives a
breakpoint and watch interface close to uVision's.

---

## Part 6: Host tooling

Separate from the firmware and unrelated to any of the above. `tools/` has its
own `CMakeLists.txt` because it builds for the PC, not the target.

```
cmake -S tools -B build
cmake --build build
py tools\maze_sim.py --sweep 500
```

`log_plot.py` needs no build at all:

```
py tools\log_plot.py capture.txt --plot ir.png
```

---

## Everyday commands

```
cmake --build build-fw                    build
cmake --build build-fw --target flash     build and flash
cmake --build build-fw --target gdbserver debug
```

Reconfigure only after editing `CMakeLists.txt` or adding a source file.

## Keeping both builds

Nothing here touches the uVision project. Build on the command line, or open
Keil and press F7; both produce a working binary from the same sources.

Add the generated directories to `.gitignore`:

```
build/
build-fw/
```

## If something breaks

| Symptom | Cause |
|---|---|
| `arm-none-eabi-gcc: not found` | toolchain not on PATH; pass `-DARM_TOOLCHAIN_DIR` |
| `No such file or directory: TMPM4KyA.h` | wrong `cmsis` include path in CMakeLists |
| `undefined reference to SystemInit` | a source file is missing from `add_executable` |
| `region FLASH overflowed` | genuinely out of flash, or the linker script was edited |
| `pyocd list` shows nothing | probe not enumerating; check the cable and drivers |
| Builds and flashes, board does nothing | wrong target in pyocd, or a vector table problem |

The last row is the one to be careful with. Reflash the Keil build to confirm
the hardware is fine before assuming the GCC build is at fault.
