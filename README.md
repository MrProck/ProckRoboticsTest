# ProckRoboticsTest

FRC Robot project using the **WPILib 2026 command-based Java framework**.

## Prerequisites
- WPILib 2026 (includes Java 17 and VS Code extension)
- CTRE Phoenix 6 (Talon FX, CANcoder, Pigeon 2.0)
- REV Robotics (CANSparkMax, CANSparkFlex)
- AndyMark am-5636 CAN color sensors
- Limelight 4 camera (configured for AprilTag pipeline)
- Xbox controllers on ports 0 and 1

## Build & Deploy
```bash
# Build
./gradlew build

# Deploy to RoboRIO
./gradlew deploy
```

## Project Structure
```
src/main/java/frc/robot/
├── Main.java                  # Entry point
├── Robot.java                 # TimedRobot — mode transitions
├── RobotContainer.java        # Subsystems, commands & button bindings
├── Constants.java             # All hardware ports & tuning constants
├── LimelightHelpers.java      # Limelight NetworkTables helper library
├── util/
│   └── REVUtil.java           # Shared REV motor controller utilities
├── subsystems/
│   ├── DriveSubsystem.java    # Swerve drive with pose estimation
│   ├── SwerveModule.java      # Individual swerve module (TalonFX + CANcoder)
│   ├── IntakeSubsystem.java   # Game piece intake mechanism
│   ├── ShooterSubsystem.java  # 4-stage shooter pipeline
│   └── VisionSubsystem.java   # Limelight 4 AprilTag pose estimation
└── commands/
    ├── TeleopDriveCommand.java        # Field-centric swerve teleop
    ├── AutonomousDriveCommand.java    # Timed forward autonomous
    └── ShootCommand.java              # Shooter spin-up & feed sequence
```

## Vision / Pose Estimation (Limelight 4)

The robot uses a **Limelight 4** camera for AprilTag-based field localization:

- **MegaTag2** pose estimation fuses the Limelight's AprilTag detections with the
  robot's IMU heading for accurate 3D pose recovery.
- Vision measurements are fed into WPILib's `SwerveDrivePoseEstimator` with
  configurable standard deviations.
- Outlier rejection filters measurements that are too far from the current
  odometry estimate or have excessive heading disagreement.
- Multi-tag detections use tighter standard deviations than single-tag for
  increased trust.

### Configuration
- **Limelight name**: Set in `Constants.VisionConstants.kLimelightName`
- **Max acceptable distance**: `kMaxAcceptableDistanceMeters` (reject vision
  poses farther than this from odometry)
- **Standard deviations**: Tunable per single-tag vs multi-tag in
  `VisionConstants`

## Controls (Teleop)

| Input | Action |
|-------|--------|
| Left Stick Y (Driver) | Forward / Backward |
| Left Stick X (Driver) | Strafe Left / Right |
| Right Stick X (Driver) | Rotate |
| Right Trigger (Driver) | Progressive brake (slow mode) |
| Start (Driver) | Zero gyro heading |
| Left Stick Y (Operator) | Extend / Retract intake |
| Right Trigger (Operator) | Run intake roller |
| Right Bumper (Operator) | Eject game piece |
| Back (Operator) | Clear intake lockout |
| Left Trigger (Operator) | Shoot (spin-up + feed) |
| Left Bumper (Operator) | Reverse all shooter stages (clear jams) |

### Driver Controller (Port 0)
| Input | Action |
|-------|--------|
| Left Stick Y | Forward / Backward |
| Left Stick X | Strafe Left / Right |
| Right Stick X | Rotate |
| Right Trigger | Progressive brake (0 = full speed, fully held = full stop) |
| Start | Zero gyro heading |

### Operator Controller (Port 1)
| Input | Action |
|-------|--------|
| Left Stick Y | Extend / Retract intake arm (push forward = extend, pull back = retract) |
| Right Trigger (hold) | Run intake roller forward (blocked if intake is locked out) |
| Right Bumper (hold) | Eject game piece (roller reverse, always allowed) |
| Left Trigger (hold) | Shoot sequence — spins up flywheels, then feeds when at speed |
| Left Bumper (hold) | Reverse all shooter stages (clear jams) |
| Back | Clear intake lockout (use after ejecting wrong-color game piece) |

### Color Sensor Safety
The intake has 3 AndyMark CAN color sensors (entry, middle, exit). If **any** sensor detects red or blue (wrong alliance game piece), the intake roller is automatically locked out. Use **Right Bumper** to eject, then **Back** to clear the lockout.

## Autonomous

Selectable via SmartDashboard **Auto Chooser**:

- **Do Nothing** (default): Stops the drive
- **Drive Forward**: Drives forward at 50% speed for 2 seconds (timed fallback)
- **Just Leave**: PathPlanner-based auto to leave the community
- **Score And Leave**: PathPlanner-based auto to score a game piece and leave

## CAN Bus Map

### CANivore Bus (`"CANivore"`)

| CAN ID | Device | Motor/Sensor | Controller | Current Limit |
|--------|--------|--------------|------------|---------------|
| 1 | FL Drive Motor | Falcon 500 | TalonFX | 60 A |
| 2 | FL Steer Motor | Falcon 500 | TalonFX | 40 A |
| 3 | FR Drive Motor | Falcon 500 | TalonFX | 60 A |
| 4 | FR Steer Motor | Falcon 500 | TalonFX | 40 A |
| 5 | BL Drive Motor | Falcon 500 | TalonFX | 60 A |
| 6 | BL Steer Motor | Falcon 500 | TalonFX | 40 A |
| 7 | BR Drive Motor | Falcon 500 | TalonFX | 60 A |
| 8 | BR Steer Motor | Falcon 500 | TalonFX | 40 A |
| 9 | FL CANcoder | — | CANcoder | — |
| 10 | FR CANcoder | — | CANcoder | — |
| 11 | BL CANcoder | — | CANcoder | — |
| 12 | BR CANcoder | — | CANcoder | — |
| 13 | Pigeon 2.0 IMU | — | Pigeon2 | — |

### RIO CAN Bus

| CAN ID | Device | Motor/Sensor | Controller | Current Limit |
|--------|--------|--------------|------------|---------------|
| 14 | Intake Extension | NEO | SparkMax | 17 A |
| 15 | Intake Roller | NEO Vortex | SparkFlex | 25 A |
| 16 | Entry Color Sensor | — | am-5636 | — |
| 17 | Middle Color Sensor | — | am-5636 | — |
| 18 | Exit Color Sensor | — | am-5636 | — |
| 19 | Shooter Agitator | NEO | SparkMax | 20 A |
| 20 | Shooter Kicker | NEO Vortex | SparkFlex | 40 A |
| 21 | Shooter Pre-Shooter | NEO Vortex | SparkFlex | 40 A |
| 22 | Shooter Primary Flywheel | NEO Vortex | SparkFlex | 80 A |
| 23 | Shooter Secondary Flywheel | NEO Vortex | SparkFlex | 80 A |

## Current Limits Summary

| Subsystem | Device | Limit | Type |
|-----------|--------|-------|------|
| Swerve Drive | Drive Motors (×4) | 60 A | Supply (TalonFX) |
| Swerve Drive | Steer Motors (×4) | 40 A | Supply (TalonFX) |
| Intake | Extension Motor | 17 A | Smart (SparkMax) |
| Intake | Roller Motor | 25 A | Smart (SparkFlex) |
| Shooter | Agitator | 20 A | Smart (SparkMax) |
| Shooter | Kicker | 40 A | Smart (SparkFlex) |
| Shooter | Pre-Shooter | 40 A | Smart (SparkFlex) |
| Shooter | Primary Flywheel | 80 A | Smart (SparkFlex) |
| Shooter | Secondary Flywheel | 80 A | Smart (SparkFlex) |

> **Total worst-case current draw**: 4×60 + 4×40 + 17 + 25 + 20 + 40 + 40 + 80 + 80 = **702 A**
> (all motors stalled simultaneously — theoretical maximum, not a realistic operating scenario)

> **Note:** `SwerveModule.java` previously contained a stale comment `// Current limit — 100A` at line 82, but the actual configured value is `SwerveConstants.kDriveCurrentLimitAmps = 60`. The comment has been updated to remove the incorrect value.

## Constants Reference

> All CAN IDs and current limits are defined in [`Constants.java`](src/main/java/frc/robot/Constants.java) and applied in the corresponding subsystem constructors. If you change a CAN ID or current limit, update **both** `Constants.java` and this README.