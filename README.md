# DEVC Motion Controller

A modular motion-control research and development project for mobile robot platforms.

The repository separates the reusable motion-control core from platform-specific code, host-side simulation, middleware integration, and embedded firmware. The current implementation focuses on vehicle-frame motion commands (`vx`, `vy`, and `w0`), wheel-speed generation, feedback control, and support for multiple robot drive configurations.

> **Project status:** This repository contains working implementation code and several platform/firmware branches of development, but the top-level build and setup flow is not yet consolidated into a single reproducible build procedure.

## Features

- Vehicle-frame motion representation using linear velocity (`vx`, `vy`) and angular velocity (`w0`)
- Forward and inverse kinematics
- Four-wheel motion-control path with wheel angular velocities (`w1` ... `w4`)
- PID control implementation
- Fuzzy-control implementation available in the common control layer
- Neural-network and genetic-algorithm related experimental modules
- Motor-control and motor-driver abstraction
- Command interface support
- Host-side simulator
- Platform-specific configurations for multiple mobile robot layouts
- STM32 Nucleo-F429ZI firmware work

## Control Flow

The main motion-control path is organized around a shared `system_data` structure.

```text
Setpoint / command
       |
       v
+--------------------+
| Motion Controller  |
|                    |
| Forward Kinematics |
| Feedback Control   |
| Inverse Kinematics |
+--------------------+
       |
       v
 Wheel commands
 w1 / w2 / w3 / w4
       |
       v
+--------------------+
| Motor Control      |
+--------------------+
       |
       v
+--------------------+
| Motor Driver / HW  |
+--------------------+
```

At initialization, the motion-control layer prepares the kinematic matrices and initializes the selected controller. During each update cycle, it:

1. Limits commanded vehicle velocities.
2. Calculates vehicle feedback through forward kinematics.
3. Runs the selected feedback controller.
4. Updates the experimental neural-network stage.
5. Converts the resulting vehicle command into individual wheel commands through inverse kinematics.

The current source selects PID control by default through `USE_PID` in `common/motion_control.c`; a fuzzy-control path is also present.

## Repository Layout

```text
.
├── common/       Shared motion-control and system code
├── firmware/     Embedded firmware targets
├── lib/          Supporting libraries
├── middleware/   Middleware integration code
├── module/       Additional project modules
├── platform/     Robot/platform-specific configuration
├── simulator/    Host-side motion-controller simulator
├── Makefile      Top-level Makefile placeholder
└── README.md
```

### `common/`

The common layer contains the reusable control implementation, including:

- `system.c/.h` - shared system state and interfaces
- `motion_control.c` - motion-control update path and kinematics
- `motor_control.c` - motor-control layer
- `commander.c` - command handling
- `pid.c` - PID controller
- `fuzzy.c` - fuzzy controller
- `nn.c` - neural-network related experimental code
- `ga.c` - genetic-algorithm related experimental code
- `navigator.c` - navigation-related module
- `small-matrix-inverse/` - matrix inversion support used by the kinematic model

## Supported / Experimental Platforms

Platform-specific code currently exists for:

- `general_2wd`
- `general_4wd`
- `nexus10011`
- `omr10`

These directories allow platform geometry and hardware-specific behavior to remain separate from the common motion-control implementation.

## Simulator

A host-side simulator is available under `simulator/`.

The simulator initializes the same common motion-control and motor-control layers used by the rest of the project and runs them in a periodic loop. The current source includes POSIX threads, SDL2-related code, serial-port handling, runtime information output, and optional data logging.

The simulator source currently defines a default control-loop period of 50 ms and includes a Windows COM-port default (`COM6`). These values should be treated as development defaults and adjusted for the target environment.

### Simulator dependencies

Dependencies visible in the current source include:

- C/C++ toolchain
- POSIX threads (`pthread`)
- SDL2
- platform-specific serial-port support
- graphics support used by the simulator

A complete portable build recipe has not yet been provided at repository root.

## Embedded Firmware

The repository includes firmware work for:

```text
firmware/stm32-nucleo-f429zi/
```

This target is based on the STM32 Nucleo-144 development platform using the STM32F429ZI MCU and the integrated ST-LINK debugger/programmer.

## Core Data Model

The common `system_data` structure is used to exchange state between the control layers. It contains, among other fields:

- vehicle setpoint, control, and process states
- `x`, `y`, and `yaw`
- `vx`, `vy`, and `w0`
- forward/inverse kinematic matrices
- per-motor direction, angular velocity, RPM, and PWM data
- PID parameters
- control-loop timing information
- motor-driver communication state

This shared data model is intended to keep the controller, motor layer, simulator, and platform implementation loosely coupled.

## Development Notes

The repository is best viewed as an evolving motion-control framework rather than a packaged application. In particular:

- The root `Makefile` does not yet define build targets.
- Some modules are experimental or incomplete.
- Simulator and firmware setup are not yet unified under one toolchain.
- Hardware-specific parameters and communication settings should be verified before use on a physical robot.

A useful next step is to formalize the build system and document one validated end-to-end reference configuration (platform, controller, simulator/firmware target, and test procedure).

## Goals

The architecture is intended to make it possible to evolve the project without tightly coupling control algorithms to one robot or one execution environment. The long-term direction is to keep:

- **control algorithms** reusable,
- **platform geometry** replaceable,
- **motor drivers** replaceable,
- **simulation and embedded execution** aligned around the same common interfaces,
- and **experimental controllers** easy to add or disable.

This makes the repository suitable for motion-control experiments, embedded implementation work, and comparison of different control strategies across mobile robot platforms.
