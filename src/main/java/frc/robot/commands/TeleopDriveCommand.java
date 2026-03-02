package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_forwardSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public TeleopDriveCommand(
        DriveSubsystem driveSubsystem,
        DoubleSupplier forwardSupplier,
        DoubleSupplier rotationSupplier
    ) {
        m_driveSubsystem = driveSubsystem;
        m_forwardSupplier = forwardSupplier;
        m_rotationSupplier = rotationSupplier;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_driveSubsystem.arcadeDrive(
            m_forwardSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
