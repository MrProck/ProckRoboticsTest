package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Default teleop drive command for field-centric swerve control.
 * <p>
 * Input pipeline per axis:
 * <ol>
 *   <li>Deadband applied ({@link SwerveConstants#kDeadband})</li>
 *   <li>Blended cubic curve applied ({@link SwerveConstants#kInputCurveLinearBlend})</li>
 *   <li>Slew rate limiter applied ({@link SwerveConstants#kTeleopSlewRatePerSecond})</li>
 *   <li>Scaled to m/s or rad/s and multiplied by the slow-mode brake factor</li>
 * </ol>
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

    // Slew rate limiters smooth acceleration on each axis independently.
    // Rate is in normalized units/second (input is −1 to 1 before scaling).
    private final SlewRateLimiter m_xLimiter   = new SlewRateLimiter(SwerveConstants.kTeleopSlewRatePerSecond);
    private final SlewRateLimiter m_yLimiter   = new SlewRateLimiter(SwerveConstants.kTeleopSlewRatePerSecond);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(SwerveConstants.kTeleopSlewRatePerSecond);

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

        double xSpeedMPS = m_xLimiter.calculate(
                blendedCubicInput(MathUtil.applyDeadband(m_xSpeed.getAsDouble(), SwerveConstants.kDeadband)))
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed
            * speedMultiplier;
        double ySpeedMPS = m_yLimiter.calculate(
                blendedCubicInput(MathUtil.applyDeadband(m_ySpeed.getAsDouble(), SwerveConstants.kDeadband)))
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed
            * speedMultiplier;
        double rotRadPerSec = m_rotLimiter.calculate(
                blendedCubicInput(MathUtil.applyDeadband(m_rotation.getAsDouble(), SwerveConstants.kDeadband)))
            * SwerveConstants.kMaxAngularVelocityRadiansPerSecond * SwerveConstants.kTeleopMaxAngularSpeed
            * speedMultiplier;

        m_driveSubsystem.drive(xSpeedMPS, ySpeedMPS, rotRadPerSec, true);
    }

    /**
     * Applies a blended linear + cubic input curve to the joystick value.
     * <p>
     * Formula: {@code output = blend * x + (1 - blend) * x³}
     * <p>
     * At full deflection (±1.0) the output is always ±1.0 regardless of blend.
     * Near center, the cubic term reduces sensitivity for finer low-speed control
     * while still allowing full speed at full stick deflection.
     * <p>
     * The blend factor is configured via {@link SwerveConstants#kInputCurveLinearBlend}.
     *
     * @param input Deadband-applied joystick value in [−1, 1]
     * @return Shaped output in [−1, 1]
     */
    private double blendedCubicInput(double input) {
        double blend = SwerveConstants.kInputCurveLinearBlend;
        return blend * input + (1.0 - blend) * (input * input * input);
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
