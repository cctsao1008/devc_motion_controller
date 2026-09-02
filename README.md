# Mecanum AGV Motion Control

A modular motion-control framework developed for a Mecanum-wheel AGV platform and used as the mobile-base control subsystem in an integrated AGV and robotic-arm system.

The project separates reusable motion-control logic from platform-specific geometry, embedded firmware, middleware, motor drivers, and host-side simulation. Its main control path operates in the vehicle frame using linear velocity commands (`vx`, `vy`) and angular velocity (`w0`), then maps those commands to individual wheel speeds through kinematic and feedback-control layers.

## Historical context

This software was developed to control the motion of a physical **Mecanum-wheel AGV base**. The AGV was later used together with a robotic arm as an integrated robotic system.

The repository represents the mobile-base motion-control side of that system: vehicle-frame motion commands, wheel kinematics, feedback control, motor-control abstraction, platform configuration, simulation, and embedded execution.

The Mecanum-wheel base represents planar motion using:

- `vx` — longitudinal vehicle velocity
- `vy` — lateral vehicle velocity
- `w0` — yaw / rotational velocity

These vehicle-level commands are converted into individual wheel commands (`w1` ... `w4`) by the kinematic layer.

## Control path

```text
Higher-level AGV command / setpoint
            vx / vy / w0
                 |
                 v
+--------------------------------+
|       Vehicle Motion Control   |
|                                |
|  velocity limits               |
|  forward kinematics            |
|  feedback controller           |
|  inverse kinematics            |
+---------------+----------------+
                |
                v
       Individual wheel commands
          w1 / w2 / w3 / w4
                |
                v
+--------------------------------+
|          Motor Control         |
+---------------+----------------+
                |
                v
+--------------------------------+
|       Motor Driver / HW        |
+--------------------------------+
                |
                v
        Mecanum-wheel AGV base
```

At initialization, the motion-control layer prepares the kinematic matrices and initializes the selected controller. During each update cycle, it:

1. limits commanded vehicle velocities,
2. calculates vehicle feedback through forward kinematics,
3. runs the selected feedback controller,
4. updates the experimental neural-network stage,
5. converts the resulting vehicle command into individual wheel commands through inverse kinematics.

The source selects PID control by default through `USE_PID` in `common/motion_control.c`; a fuzzy-control path is also present.

## Features

- Mecanum-wheel AGV motion control
- Vehicle-frame planar motion representation using `vx`, `vy`, and `w0`
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
                    |   vx / vy / w0       |
                    +----------+-----------+
                               |
                               v
                    +----------------------+
                    |      common/         |
                    | motion_control       |
                    | controller / system  |
                    +----------+-----------+
                               |
                +--------------+--------------+
                |                             |
                v                             v
       +------------------+         +-------------------+
       | platform/        |         | motor_control     |
       | wheel geometry   |         | driver abstraction|
       +------------------+         +---------+---------+
                                             |
                                             v
                                    +------------------+
                                    | Embedded HW /    |
                                    | motor interface  |
                                    +--------+---------+
                                             |
                                             v
                                    +------------------+
                                    | Mecanum-wheel    |
                                    | AGV base         |
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

The design keeps reusable motion-control logic separate from robot-specific geometry and execution environment so the same basic control path can be exercised in simulation and adapted to different physical platforms.

## Repository layout

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

## Platform variants

Platform-specific code exists for:

- `general_2wd`
- `general_4wd`
- `nexus10011`
- `omr10`

These directories isolate platform geometry and hardware-specific behavior from the common motion-control implementation.

## Core data model

The shared `system_data` structure exchanges state between control layers. It contains, among other fields:

- vehicle setpoint, control, and process states
- `x`, `y`, and `yaw`
- `vx`, `vy`, and `w0`
- forward/inverse kinematic matrices
- per-motor direction, angular velocity, RPM, and PWM data
- PID parameters
- control-loop timing information
- motor-driver communication state

This shared representation connects the controller, motor layer, simulator, and platform implementation without embedding robot-specific behavior directly into each control algorithm.

## Simulator

A host-side simulator is available under `simulator/` and initializes the same common motion-control and motor-control layers in a periodic loop. The source includes POSIX threads (`pthread`), SDL2-related code, serial-port handling, runtime information output, and optional data logging.

The simulator source defines a 50 ms default control-loop period and contains a Windows COM-port default (`COM6`).

## Embedded firmware

Embedded firmware work is present under:

```text
firmware/stm32-nucleo-f429zi/
```

This target is based on the STM32 Nucleo-144 platform using the STM32F429ZI MCU and integrated ST-LINK debugger/programmer.

## Control and experimentation

The repository contains multiple control approaches:

- **PID** — default feedback controller in the common motion-control path
- **Fuzzy control** — alternative control implementation
- **Neural-network related code** — experimental stage in the update path
- **Genetic-algorithm related code** — experimental optimization/control support

## Engineering role

```text
vehicle-frame motion commands
        ↓
Mecanum-wheel kinematics
        ↓
feedback control
        ↓
motor control
        ↓
embedded implementation
        ↓
physical AGV base
        ↓
system-level AGV + robot-arm integration
```

The architecture preserves a distinct mobile-base motion-control boundary inside the larger robotic system.

## License

No explicit top-level license is identified. Bundled third-party components remain subject to their own licenses and terms.
