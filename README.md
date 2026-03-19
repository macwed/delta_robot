# Delta Robot Firmware

ESP-IDF firmware for a 3-axis delta robot based on an ESP32-S3.

The project drives three step/dir axes with a shared GPTimer-based motion engine, computes arm targets from Cartesian `x/y/z` coordinates, and exposes a small serial console for manual control during bring-up and testing.

## Features

- synchronized 3-axis step generation on ESP32-S3
- delta inverse kinematics in Cartesian space
- gravity-assisted startup with a short home reseat
- standalone step/dir operation for the motor drivers
- interactive control over USB Serial/JTAG

## Hardware Assumptions

- ESP32-S3
- 3 stepper drivers used in standalone step/dir mode
- one motor per arm
- a mechanical low stop used as the passive home position

The current firmware assumes `16` microsteps per full step and drives the `MS1/MS2` pins directly from the MCU GPIOs.

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
status
demo on
demo off
help
```

Examples:

```text
xyz 0 0 140
xyz 20 0 210
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

This repository is focused on motion control, bring-up, and manual testing. The abandoned TMC2209 UART experiment is intentionally not part of the tracked firmware path.
