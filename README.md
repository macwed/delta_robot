# Delta Robot Firmware

ESP-IDF firmware for a 3-axis delta robot based on an ESP32-S3 and three DRV8825 drivers.

The firmware drives three step/dir axes with a shared GPTimer-based motion engine, computes arm targets from Cartesian `x/y/z` coordinates, and exposes a small serial console for bring-up and manual testing.

## Features

- synchronized 3-axis step generation on ESP32-S3
- delta inverse kinematics in Cartesian space
- DRV8825 standalone step/dir control with runtime microstep selection
- passive startup homing by disabling the drivers and letting the arms fall to `HOME_ANGLE`
- interactive control over USB Serial/JTAG

## Hardware Assumptions

- ESP32-S3
- 3 DRV8825 drivers in standalone step/dir mode
- shared `EN`, `MODE0`, `MODE1`, and `MODE2` lines for all three drivers
- one motor per arm
- a mechanical low stop used as the passive home position

The current default is `4` microsteps per full step. It can be changed at runtime with the `ms` console command.

## Wiring Used By The Current Firmware

- Motor 1: `DIR=GPIO4`, `STEP=GPIO5`
- Motor 2: `DIR=GPIO6`, `STEP=GPIO7`
- Motor 3: `DIR=GPIO15`, `STEP=GPIO16`
- Shared DRV8825 control: `EN=GPIO9`, `MODE0=GPIO12`, `MODE1=GPIO11`, `MODE2=GPIO10`

## Build

```bash
idf.py build
```

## Flash And Monitor

```bash
idf.py -p <port> flash monitor
```

The console is configured for the primary USB Serial/JTAG port, so the same `monitor` session can be used for logs and commands.

## Console Commands

```text
xyz <x> <y> <z>
ms <1|2|4|8|16|32>
status
demo on
demo off
help
```

Examples:

```text
xyz 0 0 20
ms 4
status
```

## Repository Layout

```text
main/
  main.cpp          startup and task creation
  coordinator.cpp   target selection and serial console
  motor.cpp         timer-based motion engine
  delta_ik.cpp      inverse kinematics and angle-to-step conversion
```

## Current Scope

This repository is focused on motion control, bring-up, and manual testing with DRV8825 step/dir drivers. The earlier TMC2209 UART experiment has been removed from the main project path.
