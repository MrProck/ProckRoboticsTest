# ProckRoboticsTest

FRC Robot project using the **WPILib 2026 command-based Java framework**.

## Prerequisites
- WPILib 2026 (includes Java 17 and VS Code extension)
- CTRE Phoenix 6 (Talon FX, CANcoder, Pigeon 2.0)
- REV Robotics (CANSparkMax, CANSparkFlex)
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
│   └── VisionSubsystem.java   # Limelight 4 AprilTag pose estimation
└── commands/
    ├── TeleopDriveCommand.java        # Field-centric swerve teleop
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
| Input | Action |
|-------|--------|
| Left Stick Y (Driver) | Forward / Backward |
| Left Stick X (Driver) | Strafe Left / Right |
| Right Stick X (Driver) | Rotate |
| Start (Driver) | Zero gyro heading |
| Left Stick Y (Operator) | Extend / Retract intake |
| Right Trigger (Operator) | Run intake roller |
| Right Bumper (Operator) | Eject game piece |
| Back (Operator) | Clear intake lockout |

## Autonomous
Drives forward at 50% speed for 2 seconds.