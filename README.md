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
├── subsystems/
│   ├── DriveSubsystem.java    # Swerve drive with pose estimation
│   ├── SwerveModule.java      # Individual swerve module (TalonFX + CANcoder)
│   ├── IntakeSubsystem.java   # Game piece intake mechanism
│   ├── ShooterSubsystem.java  # 4-stage shooter pipeline
│   └── VisionSubsystem.java   # Limelight 4 AprilTag pose estimation
└── commands/
    ├── TeleopDriveCommand.java        # Field-centric swerve teleop
    ├── ShootCommand.java              # Shoot sequence (spin up → feed)
    └── AutonomousDriveCommand.java    # Timed forward autonomous
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

| Option | Description |
|--------|-------------|
| **Do Nothing** (default) | Stops the drive immediately |
| **Drive Forward** | Drives forward at 50% speed for 2 seconds |
| **Just Leave** | PathPlanner auto — drives off the starting zone |
| **Score And Leave** | PathPlanner auto — scores a game piece, then leaves |