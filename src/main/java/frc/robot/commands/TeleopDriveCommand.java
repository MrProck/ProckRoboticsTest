package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Default teleop drive command for field-centric swerve control.
 * <p>
 * The driver's right trigger acts as a progressive brake: pulling the trigger
 * proportionally reduces drive and rotation speed. Fully depressing the trigger
 * stops the robot entirely.
 */
public class TeleopDriveCommand extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rotation;
    private final DoubleSupplier m_slowMode;

    /**
     * @param driveSubsystem The swerve drive subsystem
     * @param xSpeed         Forward/backward input (−1 to 1)
     * @param ySpeed         Left/right strafe input (−1 to 1)
     * @param rotation       Rotation input (−1 to 1)
     * @param slowMode       Slow-mode input (0 = full speed, 1 = full stop)
     */
    public TeleopDriveCommand(
        DriveSubsystem driveSubsystem,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed,
        DoubleSupplier rotation,
        DoubleSupplier slowMode
    ) {
        m_driveSubsystem = driveSubsystem;
        m_xSpeed   = xSpeed;
        m_ySpeed   = ySpeed;
        m_rotation = rotation;
        m_slowMode = slowMode;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Calculate progressive brake: 0.0 trigger = full speed, 1.0 trigger = full stop
        double speedMultiplier = 1.0 - MathUtil.clamp(m_slowMode.getAsDouble(), 0.0, 1.0);

        double xSpeedMPS = MathUtil.applyDeadband(m_xSpeed.getAsDouble(), SwerveConstants.kDeadband)
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed
            * speedMultiplier;
        double ySpeedMPS = MathUtil.applyDeadband(m_ySpeed.getAsDouble(), SwerveConstants.kDeadband)
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed
            * speedMultiplier;
        double rotRadPerSec = MathUtil.applyDeadband(m_rotation.getAsDouble(), SwerveConstants.kDeadband)
            * SwerveConstants.kMaxAngularVelocityRadiansPerSecond * SwerveConstants.kTeleopMaxAngularSpeed
            * speedMultiplier;

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
