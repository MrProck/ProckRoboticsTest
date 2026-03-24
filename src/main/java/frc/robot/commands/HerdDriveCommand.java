package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Drive command that auto-orients the robot based on the direction of travel.
 *
 * <p>Two modes:
 * <ul>
 *   <li><b>WIDE</b> — rotates so the closest wide side (0° or 180°) faces the
 *       direction of motion, for herding balls. When stationary for 1 second,
 *       snaps to the nearest of 0° or 180° field-relative.</li>
 *   <li><b>NARROW</b> — rotates so the closest corner (45°/135°/225°/315°) faces
 *       the direction of motion, for pushing through tight spaces. When stationary
 *       for 1 second, snaps to the nearest of 90° or 270° field-relative.</li>
 * </ul>
 *
 * <p>Translation is still fully controlled by the driver (left stick + right trigger throttle).
 * Only the heading (rotation) is automatically managed.
 */
public class HerdDriveCommand extends Command {

    public enum Mode {
        WIDE,
        NARROW
    }

    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_throttle;
    private final Mode m_mode;

    private final PIDController m_headingPID;
    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(SwerveConstants.kTeleopSlewRatePerSecond);
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(SwerveConstants.kTeleopSlewRatePerSecond);
    private final SlewRateLimiter m_throttleXLimiter = new SlewRateLimiter(SwerveConstants.kThrottleXSlewRatePerSecond);

    private final Timer m_stationaryTimer = new Timer();
    private static final double kStationaryThreshold = 0.1; // joystick magnitude below this = stationary
    private static final double kStationaryTimeSeconds = 1.0;

    // Heading PID tuning (same gains as orbital mode)
    private static final double kHeadingP = 5.5;
    private static final double kHeadingI = 0.0;
    private static final double kHeadingD = 0.0;
    private static final double kMaxRotationRadPerSec = Math.PI; // 180 °/s

    // Corner angle computed from robot geometry (27.25" wide × 12.5" deep)
    // atan2(trackWidth/2, wheelBase/2) ≈ 65.4°
    private static final double kCornerAngleDeg = Math.toDegrees(Math.atan2(
            SwerveConstants.kTrackWidthMeters / 2.0,
            SwerveConstants.kWheelBaseMeters / 2.0));

    /**
     * @param driveSubsystem The swerve drive subsystem.
     * @param xSpeed         Forward/backward input (−1 to 1).
     * @param ySpeed         Left/right strafe input (−1 to 1).
     * @param throttle       Accelerator input (0 = stopped, 1 = full speed).
     * @param mode           WIDE or NARROW orientation mode.
     */
    public HerdDriveCommand(
            DriveSubsystem driveSubsystem,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier throttle,
            Mode mode) {

        m_driveSubsystem = driveSubsystem;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_throttle = throttle;
        m_mode = mode;

        m_headingPID = new PIDController(kHeadingP, kHeadingI, kHeadingD);
        m_headingPID.enableContinuousInput(-180.0, 180.0);
        m_headingPID.setTolerance(2.0);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_headingPID.reset();
        m_stationaryTimer.restart();
        m_throttleXLimiter.reset(0.0);
    }

    @Override
    public void execute() {
        // ---- Throttle remap ----
        double trigger = MathUtil.clamp(m_throttle.getAsDouble(), 0.0, 1.0);
        double min = SwerveConstants.kThrottleMinFraction;
        double speedMultiplier = min + (1.0 - min) * trigger;
        double xSpeedMultiplier = m_throttleXLimiter.calculate(speedMultiplier);

        // ---- Joystick input with deadband + slew rate ----
        double rawX = m_xSpeed.getAsDouble();
        double rawY = m_ySpeed.getAsDouble();
        double inputX = MathUtil.applyDeadband(rawX, SwerveConstants.kDeadband);
        double inputY = MathUtil.applyDeadband(rawY, SwerveConstants.kDeadband);

        double smoothX = m_xLimiter.calculate(inputX);
        double smoothY = m_yLimiter.calculate(inputY);

        double xSpeedMPS = smoothX * SwerveConstants.kMaxDriveSpeedMetersPerSecond
                * SwerveConstants.kTeleopMaxDriveSpeed * xSpeedMultiplier;
        double ySpeedMPS = smoothY * SwerveConstants.kMaxDriveSpeedMetersPerSecond
                * SwerveConstants.kTeleopMaxDriveSpeed * speedMultiplier;

        // ---- Determine if stationary ----
        double stickMagnitude = Math.hypot(inputX, inputY);
        boolean isMoving = stickMagnitude > kStationaryThreshold;

        if (isMoving) {
            m_stationaryTimer.restart();
        }
        boolean stationaryLongEnough = m_stationaryTimer.hasElapsed(kStationaryTimeSeconds);

        // ---- Compute desired heading ----
        double currentHeadingDeg = m_driveSubsystem.getHeading().getDegrees();
        double desiredHeadingDeg;

        if (isMoving) {
            // Travel direction from joystick (field-relative angle in degrees)
            double travelAngleDeg = Math.toDegrees(Math.atan2(inputY, inputX));
            desiredHeadingDeg = snapToMode(travelAngleDeg, currentHeadingDeg);
        } else if (stationaryLongEnough) {
            desiredHeadingDeg = snapToStationaryMode(currentHeadingDeg);
        } else {
            // Not moving, but haven't been still long enough — hold current heading
            desiredHeadingDeg = currentHeadingDeg;
        }

        // ---- PID → rotation speed ----
        double rawRotationDeg = m_headingPID.calculate(currentHeadingDeg, desiredHeadingDeg);
        double rotationRadPerSec = MathUtil.clamp(
                Math.toRadians(rawRotationDeg),
                -kMaxRotationRadPerSec,
                kMaxRotationRadPerSec);

        if (m_headingPID.atSetpoint()) {
            rotationRadPerSec = 0.0;
        }

        m_driveSubsystem.drive(xSpeedMPS, ySpeedMPS, rotationRadPerSec, true);

        // ---- Telemetry ----
        SmartDashboard.putString("Herd/Mode", m_mode.name());
        SmartDashboard.putNumber("Herd/DesiredHeading", desiredHeadingDeg);
        SmartDashboard.putBoolean("Herd/Stationary", stationaryLongEnough);
    }

    /**
     * Snaps the travel angle to the closest candidate heading for the active mode.
     *
     * <p>WIDE mode candidates: travelAngle + {0, 180} (wide side faces travel direction).
     * <p>NARROW mode candidates: travelAngle + {45, 135, 225, 315} (corner faces travel direction).
     */
    private double snapToMode(double travelAngleDeg, double currentHeadingDeg) {
        double[] offsets;
        if (m_mode == Mode.WIDE) {
            offsets = new double[]{0.0, 180.0};
        } else {
            // Offsets derived from actual robot diagonal: ±cornerAngle, ±(180-cornerAngle)
            double c = kCornerAngleDeg;
            offsets = new double[]{c, -c, 180.0 - c, -(180.0 - c)};
        }
        return closestCandidate(travelAngleDeg, offsets, currentHeadingDeg);
    }

    /**
     * Snaps to the closest stationary heading.
     *
     * <p>WIDE stationary: nearest of {0, 180} field-relative.
     * <p>NARROW stationary: nearest of {90, 270} field-relative.
     */
    private double snapToStationaryMode(double currentHeadingDeg) {
        double[] candidates;
        if (m_mode == Mode.WIDE) {
            candidates = new double[]{0.0, 180.0};
        } else {
            candidates = new double[]{90.0, 270.0};
        }

        double best = candidates[0];
        double bestError = Math.abs(wrapDeg(currentHeadingDeg - candidates[0]));
        for (int i = 1; i < candidates.length; i++) {
            double error = Math.abs(wrapDeg(currentHeadingDeg - candidates[i]));
            if (error < bestError) {
                bestError = error;
                best = candidates[i];
            }
        }
        return best;
    }

    /**
     * Given a base angle and an array of offsets, returns the candidate (base + offset)
     * that is closest to the current heading (minimizes rotation).
     */
    private double closestCandidate(double baseAngleDeg, double[] offsets, double currentHeadingDeg) {
        double best = baseAngleDeg + offsets[0];
        double bestError = Math.abs(wrapDeg(currentHeadingDeg - best));
        for (int i = 1; i < offsets.length; i++) {
            double candidate = baseAngleDeg + offsets[i];
            double error = Math.abs(wrapDeg(currentHeadingDeg - candidate));
            if (error < bestError) {
                bestError = error;
                best = candidate;
            }
        }
        return best;
    }

    /** Wraps an angle to [−180, 180). */
    private static double wrapDeg(double deg) {
        deg = deg % 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
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
