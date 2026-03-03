package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousDriveCommand extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final Timer m_timer = new Timer();

    public AutonomousDriveCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        // robot-relative (fieldRelative = false) — drive straight forward
        m_driveSubsystem.drive(
            AutoConstants.kAutoDriveSpeed * SwerveConstants.kMaxDriveSpeedMetersPerSecond,
            0, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopModules();
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(AutoConstants.kAutoDriveTimeSeconds);
    }
}
