package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rotation;

    public TeleopDriveCommand(
        DriveSubsystem driveSubsystem,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed,
        DoubleSupplier rotation
    ) {
        m_driveSubsystem = driveSubsystem;
        m_xSpeed   = xSpeed;
        m_ySpeed   = ySpeed;
        m_rotation = rotation;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeedMPS = MathUtil.applyDeadband(m_xSpeed.getAsDouble(), SwerveConstants.kDeadband)
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed;
        double ySpeedMPS = MathUtil.applyDeadband(m_ySpeed.getAsDouble(), SwerveConstants.kDeadband)
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed;
        double rotRadPerSec = MathUtil.applyDeadband(m_rotation.getAsDouble(), SwerveConstants.kDeadband)
            * SwerveConstants.kMaxAngularVelocityRadiansPerSecond * SwerveConstants.kTeleopMaxAngularSpeed;
        m_driveSubsystem.drive(xSpeedMPS, ySpeedMPS, rotRadPerSec, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
