package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
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
 * <p>Left stick X controls tangential velocity (orbit CW/CCW around hub).
 * Left stick Y (via {@code radiusInput}) adjusts the target orbit radius in/out.
 * The right-trigger accelerator from {@link TeleopDriveCommand} is preserved
 * as a linear speed multiplier (0 = stopped, 1 = full speed).
 */
public class OrbitalDriveCommand extends Command {

    private final DriveSubsystem  m_driveSubsystem;
    private final DoubleSupplier  m_xSpeed;
    private final DoubleSupplier  m_ySpeed;
    private final DoubleSupplier  m_throttle;
    private final DoubleSupplier  m_radiusInput;

    private final PIDController m_headingPID;

    // Slew rate limiter on the X-axis throttle multiplier only.
    // Limits forward/backward acceleration to prevent tipping on the narrow 12.5" wheelbase.
    // Y-axis (strafe) uses the unramped throttle for instant responsiveness.
    private final SlewRateLimiter m_throttleXLimiter = new SlewRateLimiter(SwerveConstants.kThrottleXSlewRatePerSecond);

    // Current target orbit radius, seeded from actual distance on initialize()
    private double m_targetRadiusMeters = OrbitalConstants.kOrbitDefaultRadiusMeters;

    /**
     * Creates a new OrbitalDriveCommand.
     *
     * @param driveSubsystem The swerve drive subsystem.
     * @param xSpeed         Forward/backward translation supplier (-1 to 1); used as tangential speed.
     * @param ySpeed         Left/right strafe supplier (-1 to 1).
     * @param throttle       Accelerator supplier (0 = stopped, 1 = full speed).
     * @param radiusInput    Radial in/out supplier (-1 to 1); positive = move away from hub.
     */
    public OrbitalDriveCommand(
            DriveSubsystem driveSubsystem,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier throttle,
            DoubleSupplier radiusInput) {

        m_driveSubsystem = driveSubsystem;
        m_xSpeed         = xSpeed;
        m_ySpeed         = ySpeed;
        m_throttle       = throttle;
        m_radiusInput    = radiusInput;

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

        // Seed target radius from actual distance to hub to avoid sudden jumps
        Translation2d hub = getHubPosition();
        Pose2d robotPose = m_driveSubsystem.getPose();
        double actualDistance = robotPose.getTranslation().getDistance(hub);
        m_targetRadiusMeters = MathUtil.clamp(
                actualDistance,
                OrbitalConstants.kOrbitMinRadiusMeters,
                OrbitalConstants.kOrbitMaxRadiusMeters);
    }

    @Override
    public void execute() {
        // ---- Determine alliance-correct hub position ----
        Translation2d hub = getHubPosition();

        // ---- Current robot pose (from pose estimator) ----
        Pose2d robotPose = m_driveSubsystem.getPose();

        // ---- Vector from hub to robot ----
        double dx = robotPose.getX() - hub.getX();
        double dy = robotPose.getY() - hub.getY();
        double actualDistance = Math.sqrt(dx * dx + dy * dy);

        // ---- Adjust target radius via radial input ----
        double radialInput = MathUtil.applyDeadband(m_radiusInput.getAsDouble(), SwerveConstants.kDeadband);
        m_targetRadiusMeters += radialInput * OrbitalConstants.kOrbitRadiusAdjustRate * 0.02;
        m_targetRadiusMeters = MathUtil.clamp(
                m_targetRadiusMeters,
                OrbitalConstants.kOrbitMinRadiusMeters,
                OrbitalConstants.kOrbitMaxRadiusMeters);

        // ---- Compute desired heading to hub (robot faces hub) ----
        double desiredHeadingDeg = Math.toDegrees(Math.atan2(-dy, -dx));

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

        // ---- Compute unit tangent vector (perpendicular to robot→hub, CCW positive) ----
        // Unit radial vector (hub → robot): (dx, dy) / actualDistance
        // Tangent (CCW): (-dy, dx) / actualDistance
        double tangentX = 0.0;
        double tangentY = 0.0;
        double radialCorrX = 0.0;
        double radialCorrY = 0.0;
        if (actualDistance > 0.01) {
            double unitRadialX = dx / actualDistance;
            double unitRadialY = dy / actualDistance;
            double unitTangentX = -unitRadialY;
            double unitTangentY =  unitRadialX;

            // ---- Tangential velocity from left stick X ----
            double trigger = MathUtil.clamp(m_throttle.getAsDouble(), 0.0, 1.0);
            double tangentialInput = MathUtil.applyDeadband(m_xSpeed.getAsDouble(), SwerveConstants.kDeadband);
            double tangentialSpeed = tangentialInput * OrbitalConstants.kOrbitMaxTangentialSpeedMPS * trigger;
            tangentX = unitTangentX * tangentialSpeed;
            tangentY = unitTangentY * tangentialSpeed;

            // ---- Radial correction: P-control to maintain target orbit radius ----
            double radialError = actualDistance - m_targetRadiusMeters;
            double radialSpeed = -radialError * OrbitalConstants.kOrbitRadialP;
            radialCorrX = unitRadialX * radialSpeed;
            radialCorrY = unitRadialY * radialSpeed;
        }

        // ---- Combine tangential + radial into field-relative vx/vy ----
        double vxMPS = tangentX + radialCorrX;
        double vyMPS = tangentY + radialCorrY;

        // ---- Drive (field-centric) ----
        m_driveSubsystem.drive(vxMPS, vyMPS, rotationRadPerSec, true);

        // ---- Telemetry ----
        if (Preferences.getBoolean("Drive/Verbose Telemetry", false)) {
            SmartDashboard.putNumber("Orbital/DesiredHeadingDeg", desiredHeadingDeg);
            SmartDashboard.putNumber("Orbital/CurrentHeadingDeg", currentHeadingDeg);
            SmartDashboard.putNumber("Orbital/HeadingErrorDeg",
                    desiredHeadingDeg - currentHeadingDeg);
            SmartDashboard.putBoolean("Orbital/AtSetpoint", m_headingPID.atSetpoint());
            SmartDashboard.putNumber("Orbital/HubX", hub.getX());
            SmartDashboard.putNumber("Orbital/HubY", hub.getY());
            SmartDashboard.putNumber("Orbital/TargetRadius", m_targetRadiusMeters);
            SmartDashboard.putNumber("Orbital/DistToHub", actualDistance);
            SmartDashboard.putNumber("Orbital/RadiusError", actualDistance - m_targetRadiusMeters);
        }
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
}
