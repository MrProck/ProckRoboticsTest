package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Default teleop drive command for field-centric swerve control.
 * <p>
 * Input pipeline per axis:
 * <ol>
 *   <li>Deadband applied ({@link SwerveConstants#kDeadband})</li>
 *   <li>Blended cubic curve applied ({@link SwerveConstants#kInputCurveLinearBlend})</li>
 *   <li>Slew rate limiter applied ({@link SwerveConstants#kTeleopSlewRatePerSecond})</li>
 *   <li>Scaled to m/s or rad/s and multiplied by the throttle accelerator factor</li>
 * </ol>
 * <p>
 * The driver's right trigger acts as a linear accelerator:
 * the robot is stopped when the trigger is not pressed and linearly ramps up
 * to full speed as the trigger is fully pulled.
 * <p>
 * Hold the Back button to switch to robot-centric driving. Release to return
 * to field-centric mode.
 */
public class TeleopDriveCommand extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier  m_xSpeed;
    private final DoubleSupplier  m_ySpeed;
    private final DoubleSupplier  m_rotation;
    private final DoubleSupplier  m_throttle;
    private final BooleanSupplier m_robotCentric;

    // Slew rate limiters smooth acceleration on each axis independently.
    // Rate is in normalized units/second (input is −1 to 1 before scaling).
    private final SlewRateLimiter m_xLimiter   = new SlewRateLimiter(SwerveConstants.kTeleopSlewRatePerSecond);
    private final SlewRateLimiter m_yLimiter   = new SlewRateLimiter(SwerveConstants.kTeleopSlewRatePerSecond);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(SwerveConstants.kTeleopSlewRatePerSecond);

    // Slew rate limiter on the X-axis throttle multiplier only.
    // Limits forward/backward acceleration to prevent tipping on the narrow 12.5" wheelbase.
    // Y-axis (strafe) and rotation use the unramped throttle for instant responsiveness.
    private final SlewRateLimiter m_throttleXLimiter = new SlewRateLimiter(SwerveConstants.kThrottleXSlewRatePerSecond);

    /**
     * @param driveSubsystem The swerve drive subsystem
     * @param xSpeed         Forward/backward input (−1 to 1)
     * @param ySpeed         Left/right strafe input (−1 to 1)
     * @param rotation       Rotation input (−1 to 1)
     * @param throttle       Accelerator input (0 = stopped, 1 = full speed); linearly
     *                       scales all drive outputs
     * @param robotCentric   When {@code true}, drives robot-centric instead of field-centric
     */
    public TeleopDriveCommand(
        DriveSubsystem driveSubsystem,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed,
        DoubleSupplier rotation,
        DoubleSupplier throttle,
        BooleanSupplier robotCentric
    ) {
        m_driveSubsystem = driveSubsystem;
        m_xSpeed       = xSpeed;
        m_ySpeed       = ySpeed;
        m_rotation     = rotation;
        m_throttle     = throttle;
        m_robotCentric = robotCentric;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_throttleXLimiter.reset(0.0);
    }

    @Override
    public void execute() {
        // Throttle remap: trigger [0, 1] → [kThrottleMinFraction, 1.0]
        // This ensures the robot drives at kThrottleMinFraction of max speed even
        // with no trigger input, while still allowing full speed at full trigger.
        double trigger = MathUtil.clamp(m_throttle.getAsDouble(), 0.0, 1.0);
        double min = SwerveConstants.kThrottleMinFraction;
        double speedMultiplier = min + (1.0 - min) * trigger;
        // X-axis throttle is ramped to prevent tipping on the narrow 12.5" wheelbase.
        // Y-axis and rotation use the raw speedMultiplier for instant responsiveness.
        double xSpeedMultiplier = m_throttleXLimiter.calculate(speedMultiplier);

        double rawX = m_xSpeed.getAsDouble();
        double rawY = m_ySpeed.getAsDouble();
        double rawRot = m_rotation.getAsDouble();

        double xSpeedMPS = m_xLimiter.calculate(
                squaredInput(MathUtil.applyDeadband(rawX, SwerveConstants.kDeadband)))
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed
            * xSpeedMultiplier;
        double ySpeedMPS = m_yLimiter.calculate(
                squaredInput(MathUtil.applyDeadband(rawY, SwerveConstants.kDeadband)))
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed
            * speedMultiplier;
        double rotRadPerSec = m_rotLimiter.calculate(
                squaredInput(MathUtil.applyDeadband(rawRot, SwerveConstants.kDeadband)))
            * SwerveConstants.kMaxAngularVelocityRadiansPerSecond * SwerveConstants.kTeleopMaxAngularSpeed
            * speedMultiplier;

        if (Preferences.getBoolean("Drive/Verbose Telemetry", false)) {
            SmartDashboard.putNumber("Joystick Raw X", rawX);
            SmartDashboard.putNumber("Joystick Raw Y", rawY);
            SmartDashboard.putNumber("Joystick Raw Rot", rawRot);
            SmartDashboard.putNumber("Cmd xSpeed (m/s)", xSpeedMPS);
            SmartDashboard.putNumber("Cmd ySpeed (m/s)", ySpeedMPS);
            SmartDashboard.putNumber("Cmd rot (rad/s)", rotRadPerSec);
        }

        boolean fieldRelative = !m_robotCentric.getAsBoolean();
        SmartDashboard.putBoolean("Drive/Robot Centric", !fieldRelative);
        SmartDashboard.putString("Drive/Drive Mode", fieldRelative ? "Field Centric" : "Robot Centric");
        m_driveSubsystem.drive(xSpeedMPS, ySpeedMPS, rotRadPerSec, fieldRelative);
    }

    /**
     * Squares the magnitude of the joystick input while preserving sign.
     * <p>
     * Formula: {@code output = input * |input|}
     * <p>
     * At full deflection (±1.0) the output is always ±1.0.
     * Near center, sensitivity is reduced for finer low-speed control.
     *
     * @param input Deadband-applied joystick value in [−1, 1]
     * @return Shaped output in [−1, 1]
     */
    private double squaredInput(double input) {
        return input * Math.abs(input);
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
