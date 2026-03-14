package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OrbitalConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Orbital drive mode for the 2026 FRC REBUILT season.
 *
 * <p>Locks the robot's heading toward the field hub (the central hexagonal
 * field element) while the driver retains full translation control.
 * The hub target position is automatically mirrored for the Red alliance.
 *
 * <p>Hub center coordinates are derived from the official WPILib
 * 2026-rebuilt-welded AprilTag field layout JSON:
 * <ul>
 *   <li>Blue hub: (4.8755, 4.0346) m</li>
 *   <li>Red  hub: (11.9155, 4.0346) m</li>
 * </ul>
 *
 * <p>Translation inputs pass through unchanged (field-centric swerve).
 * The right-trigger brake from {@link TeleopDriveCommand} is preserved
 * as a proportional speed multiplier (0 = full speed, 1 = stopped).
 */
public class OrbitalDriveCommand extends Command {

    private final DriveSubsystem  m_driveSubsystem;
    private final DoubleSupplier  m_xSpeed;
    private final DoubleSupplier  m_ySpeed;
    private final DoubleSupplier  m_throttle;

    private final PIDController m_headingPID;

    // Slew rate limiter on the X-axis throttle multiplier only.
    // Limits forward/backward acceleration to prevent tipping on the narrow 12.5" wheelbase.
    // Y-axis (strafe) uses the unramped throttle for instant responsiveness.
    private final SlewRateLimiter m_throttleXLimiter = new SlewRateLimiter(SwerveConstants.kThrottleXSlewRatePerSecond);

    /**
     * Creates a new OrbitalDriveCommand.
     *
     * @param driveSubsystem The swerve drive subsystem.
     * @param xSpeed         Forward/backward translation supplier (−1 to 1).
     * @param ySpeed         Left/right strafe supplier (−1 to 1).
     * @param throttle       Brake supplier (0 = full speed, 1 = stopped).
     */
    public OrbitalDriveCommand(
            DriveSubsystem driveSubsystem,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier throttle) {

        m_driveSubsystem = driveSubsystem;
        m_xSpeed         = xSpeed;
        m_ySpeed         = ySpeed;
        m_throttle       = throttle;

        m_headingPID = new PIDController(
                OrbitalConstants.kOrbitalP,
                OrbitalConstants.kOrbitalI,
                OrbitalConstants.kOrbitalD);

        // Continuous input so the PID wraps correctly across the ±180° boundary
        m_headingPID.enableContinuousInput(-180.0, 180.0);
        m_headingPID.setTolerance(OrbitalConstants.kOrbitalToleranceDegrees);

        addRequirements(driveSubsystem);
    }

    // -------------------------------------------------------------------------
    // Command lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void initialize() {
        m_headingPID.reset();
        m_throttleXLimiter.reset(0.0);
    }

    @Override
    public void execute() {
        // ---- Determine alliance-correct hub position ----
        Translation2d hub = getHubPosition();

        // ---- Current robot pose (from pose estimator) ----
        Pose2d robotPose = m_driveSubsystem.getPose();

        // ---- Compute desired heading to hub ----
        double dx = hub.getX() - robotPose.getX();
        double dy = hub.getY() - robotPose.getY();
        double desiredHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));

        // ---- Current robot heading ----
        double currentHeadingDeg = m_driveSubsystem.getHeading().getDegrees();

        // ---- PID output → rotation speed ----
        double rawRotation = m_headingPID.calculate(currentHeadingDeg, desiredHeadingDeg);
        double rotationRadPerSec = MathUtil.clamp(
                rawRotation,
                -OrbitalConstants.kOrbitalMaxRotationRadPerSec,
                 OrbitalConstants.kOrbitalMaxRotationRadPerSec);

        // Suppress tiny PID corrections when already on target
        if (m_headingPID.atSetpoint()) {
            rotationRadPerSec = 0.0;
        }

        // ---- Translation inputs (pass-through, with brake) ----
        double speedMult = 1.0 - MathUtil.clamp(m_throttle.getAsDouble(), 0.0, 1.0);
        // X-axis throttle is ramped to prevent tipping on the narrow 12.5" wheelbase.
        // Y-axis uses the raw speedMult for instant responsiveness.
        double xSpeedMult = m_throttleXLimiter.calculate(speedMult);

        double xSpeedMPS = squaredInput(
                MathUtil.applyDeadband(m_xSpeed.getAsDouble(), SwerveConstants.kDeadband))
                * SwerveConstants.kMaxDriveSpeedMetersPerSecond
                * SwerveConstants.kTeleopMaxDriveSpeed
                * xSpeedMult;

        double ySpeedMPS = squaredInput(
                MathUtil.applyDeadband(m_ySpeed.getAsDouble(), SwerveConstants.kDeadband))
                * SwerveConstants.kMaxDriveSpeedMetersPerSecond
                * SwerveConstants.kTeleopMaxDriveSpeed
                * speedMult;

        // ---- Drive (field-centric) ----
        m_driveSubsystem.drive(xSpeedMPS, ySpeedMPS, rotationRadPerSec, true);

        // ---- Telemetry ----
        SmartDashboard.putNumber("Orbital/DesiredHeadingDeg", desiredHeadingDeg);
        SmartDashboard.putNumber("Orbital/CurrentHeadingDeg", currentHeadingDeg);
        SmartDashboard.putNumber("Orbital/HeadingErrorDeg",
                desiredHeadingDeg - currentHeadingDeg);
        SmartDashboard.putBoolean("Orbital/AtSetpoint", m_headingPID.atSetpoint());
        SmartDashboard.putNumber("Orbital/HubX", hub.getX());
        SmartDashboard.putNumber("Orbital/HubY", hub.getY());
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button is released
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /**
     * Returns the hub center position in WPILib field coordinates, mirrored
     * for the Red alliance.
     *
     * <p>If the alliance is unknown (e.g. practice mode), defaults to Blue.
     */
    private Translation2d getHubPosition() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return new Translation2d(OrbitalConstants.kHubRedX, OrbitalConstants.kHubY);
        }
        return new Translation2d(OrbitalConstants.kHubBlueX, OrbitalConstants.kHubY);
    }

    /**
     * Applies a squared input curve preserving direction sign.
     * Reduces sensitivity near center while leaving full deflection unchanged.
     */
    private double squaredInput(double input) {
        return Math.copySign(input * input, input);
    }
}
