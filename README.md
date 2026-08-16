# DEVC Motion Controller

A modular mobile-robot motion-control framework originally developed for an AGV platform and used as part of an integrated AGV and robotic-arm system.

The project separates reusable motion-control logic from platform-specific geometry, embedded firmware, middleware, motor drivers, and host-side simulation. Its main control path operates in the vehicle frame using linear velocity commands (`vx`, `vy`) and angular velocity (`w0`), then converts them into per-wheel commands through kinematic and feedback-control layers.

> **Project status:** Legacy / reference project. The repository contains working implementation code, platform variants, simulator code, and embedded firmware work, but the top-level build and setup flow is not consolidated into a single reproducible procedure.

## Historical Context

This software was developed in the context of a physical AGV platform and later used within an integrated AGV and robot-arm system.

The repository represents the **mobile-platform motion-control side** of that system: vehicle motion commands, kinematics, feedback control, motor-control abstraction, platform configuration, and embedded execution.

It should not be interpreted as the complete software stack for the robot arm or the entire integrated robotic system.

## What the Project Does

The core software turns higher-level vehicle motion commands into wheel-level motor commands while keeping control algorithms and platform geometry reasonably independent.

```text
Higher-level command / setpoint
             |
             v
+----------------------------+
|   Vehicle Motion Control   |
|                            |
|  velocity limits           |
|  forward kinematics        |
|  feedback controller       |
|  inverse kinematics        |
+-------------+--------------+
              |
              v
        Wheel commands
       w1 / w2 / w3 / w4
              |
              v
+----------------------------+
|       Motor Control        |
+-------------+--------------+
              |
              v
+----------------------------+
|    Motor Driver / HW       |
+----------------------------+
```

At initialization, the motion-control layer prepares the kinematic matrices and initializes the selected controller. During each update cycle, it:

1. limits commanded vehicle velocities,
2. calculates vehicle feedback through forward kinematics,
3. runs the selected feedback controller,
4. updates the experimental neural-network stage,
5. converts the resulting vehicle command into individual wheel commands through inverse kinematics.

The current source selects PID control by default through `USE_PID` in `common/motion_control.c`; a fuzzy-control path is also present.

## Features

- Vehicle-frame motion representation using `vx`, `vy`, and `w0`
- Forward and inverse kinematics
- Four-wheel motion-control path using `w1` ... `w4`
- PID feedback control
- Fuzzy-control implementation
- Neural-network and genetic-algorithm related experimental modules
- Motor-control and motor-driver abstraction
- Command-handling layer
- Multiple robot-platform configurations
- Host-side simulator
- STM32 Nucleo-F429ZI embedded firmware work

## Architecture

```text
                   +----------------------+
                   | Command / Setpoint   |
                   +----------+-----------+
                              |
                              v
                   +----------------------+
                   |   common/            |
                   | motion_control       |
                   | controller / system  |
                   +----------+-----------+
                              |
               +--------------+--------------+
               |                             |
               v                             v
      +------------------+         +------------------+
      | platform/        |         | motor_control    |
      | robot geometry   |         | driver abstraction|
      +------------------+         +--------+---------+
                                           |
                                           v
                                  +------------------+
                                  | Embedded HW /    |
                                  | motor interface  |
                                  +------------------+

               Shared control logic
                       |
          +------------+-------------+
          |                          |
          v                          v
 +------------------+       +------------------+
 | simulator/       |       | firmware/        |
 | host execution   |       | STM32 target     |
 +------------------+       +------------------+
```

The design keeps the reusable motion-control core separate from robot-specific geometry and execution environment so that the same basic control path can be exercised in simulation and adapted to different physical platforms.

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

- `system.c/.h` — shared system state and interfaces
- `motion_control.c` — motion-control update path and kinematics
- `motor_control.c` — motor-control layer
- `commander.c` — command handling
- `pid.c` — PID controller
- `fuzzy.c` — fuzzy controller
- `nn.c` — neural-network related experimental code
- `ga.c` — genetic-algorithm related experimental code
- `navigator.c` — navigation-related module
- `small-matrix-inverse/` — matrix inversion support used by the kinematic model

## Supported / Experimental Platforms

Platform-specific code currently exists for:

- `general_2wd`
- `general_4wd`
- `nexus10011`
- `omr10`

These directories isolate platform geometry and hardware-specific behavior from the common motion-control implementation.

This is important for mobile-robot development because vehicle geometry, wheel layout, and hardware details can change without requiring the higher-level control architecture to be rewritten.

## Core Data Model

The shared `system_data` structure is used to exchange state between control layers. It contains, among other fields:

- vehicle setpoint, control, and process states
- `x`, `y`, and `yaw`
- `vx`, `vy`, and `w0`
- forward/inverse kinematic matrices
- per-motor direction, angular velocity, RPM, and PWM data
- PID parameters
- control-loop timing information
- motor-driver communication state

This common data model keeps the controller, motor layer, simulator, and platform implementation connected through a shared representation rather than embedding robot-specific behavior directly into each control algorithm.

## Simulator

A host-side simulator is available under `simulator/`.

The simulator initializes the same common motion-control and motor-control layers and runs them in a periodic loop. The current source includes:

- POSIX threads (`pthread`)
- SDL2-related code
- serial-port handling
- runtime information output
- optional data logging

The simulator source currently defines a default control-loop period of 50 ms and contains a Windows COM-port default (`COM6`). These should be treated as historical development defaults rather than portable configuration values.

A complete portable build recipe has not yet been provided at repository root.

## Embedded Firmware

Embedded firmware work is present under:

```text
firmware/stm32-nucleo-f429zi/
```

This target is based on the STM32 Nucleo-144 platform using the STM32F429ZI MCU and integrated ST-LINK debugger/programmer.

The firmware side demonstrates the intended transition from host-side motion-control development to execution on a physical embedded controller.

## Control and Experimentation

The repository contains multiple control approaches at different levels of maturity:

- **PID** — default feedback controller in the current common motion-control path
- **Fuzzy control** — alternative control implementation
- **Neural-network related code** — experimental stage in the update path
- **Genetic-algorithm related code** — experimental optimization/control support

These modules reflect the project’s role as both a practical mobile-robot controller and a development environment for comparing control approaches.

## Design Characteristics

The repository illustrates several architectural ideas that remain useful beyond the original AGV system:

- separation of vehicle-level control from motor-level control
- reusable kinematic and feedback-control logic
- platform geometry isolated from common algorithms
- replaceable motor-driver and hardware layers
- shared interfaces between host simulation and embedded execution
- experimental controllers that can be introduced without restructuring the full system
- support for evolving one motion-control core across multiple mobile-robot configurations

In the original application context, this structure allowed the mobile platform to serve as one subsystem within a larger robotic system while preserving a distinct motion-control boundary.

## Development Notes

The repository is best viewed as an evolving engineering project rather than a packaged library.

Known limitations include:

- the root `Makefile` does not define a complete build flow,
- some modules are experimental or incomplete,
- simulator and firmware setup are not unified under one toolchain,
- hardware-specific parameters and communication settings require verification before reuse,
- no current end-to-end reference build has been documented for modern development environments.

If the project is revived, the first priority should be reproducing one validated reference configuration before attempting major architectural changes.

A practical sequence would be:

```text
1. Select one known platform configuration
          ↓
2. Reproduce the common motion-control build
          ↓
3. Validate kinematics and controller behavior in simulation
          ↓
4. Reproduce one embedded firmware target
          ↓
5. Verify motor-driver communication and wheel feedback
          ↓
6. Run an end-to-end mobile-platform motion test
```

## Engineering Role

This repository represents more than an isolated control-algorithm experiment. It combines:

```text
control algorithms
        ↓
vehicle kinematics
        ↓
motor control
        ↓
embedded implementation
        ↓
physical mobile platform
        ↓
system-level robotic integration
```

The historical AGV and robot-arm application is therefore useful context for understanding why the project contains both reusable control abstractions and multiple execution/platform layers.

## License

No explicit top-level license is currently identified. Unless a license is added, normal copyright rules apply to the original project code and any bundled third-party components remain subject to their own licenses and terms.
