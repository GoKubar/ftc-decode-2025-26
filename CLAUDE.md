# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FTC (FIRST Tech Challenge) robot controller codebase for the **DECODE (2025-2026)** competition season, built on the FTC SDK v11. The team code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

## Build Commands

This is an Android Gradle project. Build and deploy via Android Studio or the Gradle wrapper:

```bash
# Build debug APK
./gradlew :TeamCode:assembleDebug

# Run unit tests
./gradlew :TeamCode:test

# Run a single test class
./gradlew :TeamCode:test --tests "org.firstinspires.ftc.teamcode.YourTestClass"
```

Deployment to the robot requires Android Studio's "Run" button targeting the connected Robot Controller device, or using `./gradlew :TeamCode:installDebug` over ADB.

## Architecture

### Core Abstractions

**`Robot`** (`robot/Robot.java`) — Central class that owns all subsystems and is the single entry point for all OpModes. It holds:
- `Shooter` (turret + flywheel + hood + gate)
- `PTO` (Power Take-Off: dual-purpose intake/lift motors + servo)
- `Follower` (PedroPathing path follower for autonomous)
- `GoBildaPinpointDriver` (odometry pod)
- Active `State` (current robot behavior mode)
- Active `Drivetrain` (swappable drive style)

**`State` interface** (`states/State.java`) — Defines robot behavior. Implementations: `IntakingState`, `ShootingState`, `LiftingState`, `None`. Each state has `initialize(Robot, State)` and `execute(Robot)`. States are registered via the `States` enum and created with `robot.setState(States.X)`.

**`Drivetrain` interface** (`drivetrains/Drivetrain.java`) — Swappable drive modes. Available: `Swerve`, `SwerveHeadingLock` (default in TeleOp), `AngleSwerve`, `Mecanum`, `AngleMecanum`. Swapped via `robot.setDrivetrain(Drivetrains.X)`.

### Scheduler / Command Pattern

The codebase uses **PedroPathing's Ivy scheduler** (`com.pedropathing.ivy.Scheduler`). Commands are scheduled with `Scheduler.schedule(command)` and executed each loop with `Scheduler.execute()`. Key command factories:
- `instant(() -> ...)` — runs once
- `infinite(() -> ...)` — runs every loop until cancelled
- `sequential(...)`, `parallel(...)`, `race(...)` — command groups
- `waitMs(ms)`, `waitUntil(() -> bool)`

The main loop in `Robot.init()` schedules an `infinite(this::loop)` command. All subsystem updates happen inside `Robot.loop()`.

### Shooter System

`Shooter.java` coordinates three hardware components:
- **`Turret`** — dual servos (`turretFront`, `turretBack`), maps angle ↔ servo position linearly. Offset adjusted at runtime via `Turret.turretOffsetRad`.
- **`Flywheel`** — DC motor with velocity PID.
- **`Hood`** — servo controlling launch angle.

Shooting targeting uses **`VelocityCompensationCalculator`** which performs iterative physics-based solving (bilinear interpolation lookup tables for flywheel speed and hood angle, keyed on dx/dy to goal). The goal pose is `Constants.BLUE_GOAL_POSE` (mirrored for red via `.mirror()`).

### Localization

Two modes switchable at runtime via `Robot.setLocalizationMode()`:
- **`FOLLOWER`** — PedroPathing's internal odometry (used in Auto)
- **`PINPOINT`** — GoBilda Pinpoint dead-wheel odometry (used in TeleOp)

Pose is kept consistent across both via `Constants.lastPose` and `robot.setPose()`.

### OpMode Structure

- **`Auto`** (abstract) → `BlueAuto21`, `RedAuto21`, `BlueAutoCompatibility`, `RedAutoCompatibility` — autonomous OpModes. Uses `FOLLOWER` localization. Paths are BezierLine/BezierCurve `PathChain`s built in `generatePaths()`.
- **`Tele`** (abstract) → `BlueTele`, `RedTele` — TeleOp OpModes. Uses `PINPOINT` localization. Starts in `INTAKING` state. Inherits pose from auto via `Constants.lastPose` if `Constants.lastOpModeWasAuto` is true.

### Hardware Wrappers

`util/hardware/` contains caching wrappers (`MotorEx`, `ServoEx`, `CRServoEx`) that skip hardware calls if the new value is within a tolerance of the last commanded value. Tolerances are in `Constants`.

### Performance

**PhotonCore** is enabled for parallel hardware I/O. Bulk caching is `MANUAL` — `robot.clearCaches()` must be called at the start of each loop iteration (done in OpMode loop bodies).

## Key Constants & Tuning Points

- `Constants.BLUE_GOAL_POSE` — target scoring position (red = `.mirror()`)
- `Turret.turretOffsetRad` — runtime turret trim (gamepad2 dpad left/right, ±3°)
- `VelocityCompensationCalculator.flywheelSpeeds` / `hoodServoPositions` — 2D lookup tables indexed by (dx, dy) to goal
- `PedroConstants` — all swerve drive PID coefficients and odometry pod offsets
- `Robot.singleShootTimeMillis` — gate open duration per shot
- `Shooter.transitionYValue` — y-position threshold switching between single-shot and motif-shot modes

## Gamepad Controls (TeleOp)

**Gamepad 1:**
- Right bumper: transition INTAKING → SHOOTING (or fire in SHOOTING)
- Left bumper: single shot while intaking
- B: cancel shooting, return to INTAKING
- Right trigger: run intake forward
- Left trigger: run intake reverse

**Gamepad 2:**
- D-pad left/right: adjust turret offset ±3°
- D-pad down: toggle debug telemetry
- A: reset pose to known position (color-dependent)
