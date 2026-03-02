# ProckRoboticsTest

FRC Robot project using the **WPILib 2025 command-based Java framework**.

## Prerequisites
- WPILib 2025 (includes Java 17 and VS Code extension)
- REV Robotics CANSparkMax motors (brushless)
- Xbox controller on port 0

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
├── subsystems/
│   └── DriveSubsystem.java    # 4x CANSparkMax differential drive
└── commands/
    ├── TeleopDriveCommand.java        # Xbox arcade drive
    └── AutonomousDriveCommand.java    # Timed forward autonomous
```

## Configuration
Update your **team number** in:
- `.wpilib/wpilib_preferences.json`
- `build.gradle` (`frc.getTeamOrDefault(9999)`)

Update **CAN IDs** in `Constants.java` → `DriveConstants` to match your robot wiring.

## Controls (Teleop)
| Input | Action |
|-------|--------|
| Left Stick Y | Forward / Backward |
| Right Stick X | Turn Left / Right |

## Autonomous
Drives forward at 50% speed for 2 seconds.