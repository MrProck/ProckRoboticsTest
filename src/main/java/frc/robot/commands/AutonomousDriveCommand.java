package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
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
        m_driveSubsystem.arcadeDrive(AutoConstants.kAutoDriveSpeed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopDrive();
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(AutoConstants.kAutoDriveTimeSeconds);
    }
}
