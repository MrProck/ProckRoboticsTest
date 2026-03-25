package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OrbitalConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Hub-aim drive mode for the 2026 FRC REBUILT season.
 *
 * <p>Locks the robot's heading so that the Limelight camera faces hub
 * AprilTags 9/10 (Red) or 25/26 (Blue), while the driver retains full
 * field-centric translation control with the left stick.
 *
 * <p>The desired robot heading is computed from the robot's field pose
 * to the hub center, then offset by the camera's yaw mount angle so the
 * camera barrel (not the robot nose) points directly at the hub.
 *
 * <p>Translation is passed through exactly as in normal teleop — the command
 * does NOT impose any orbital, radial, or tangential motion on its own.
 */
public class OrbitalDriveCommand extends Command {

    private final DriveSubsystem  m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final DoubleSupplier  m_xSpeed;
    private final DoubleSupplier  m_ySpeed;
    private final DoubleSupplier  m_throttle;

    private final PIDController m_headingPID;

    /**
     * Creates a new OrbitalDriveCommand.
     *
     * @param driveSubsystem  The swerve drive subsystem.
     * @param visionSubsystem The vision subsystem (provides hub translation).
     * @param xSpeed          Forward/backward supplier (-1 to 1, field-centric).
     * @param ySpeed          Left/right strafe supplier (-1 to 1, field-centric).
     * @param throttle        Right-trigger throttle supplier (0 = min speed, 1 = full speed).
     * @param unused          Ignored — kept for call-site compatibility (was radiusInput).
     */
    public OrbitalDriveCommand(
            DriveSubsystem driveSubsystem,
            VisionSubsystem visionSubsystem,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier throttle,
            DoubleSupplier unused) {

        m_driveSubsystem  = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_xSpeed          = xSpeed;
        m_ySpeed          = ySpeed;
        m_throttle        = throttle;

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
    }

    @Override
    public void execute() {
        // ---- Determine hub position (tag-based when visible, else constants) ----
        Translation2d hub = m_visionSubsystem.getHubTranslation();

        // ---- Current robot pose ----
        Pose2d robotPose = m_driveSubsystem.getPose();

        // ---- Vector from robot to hub ----
        double dx = hub.getX() - robotPose.getX();
        double dy = hub.getY() - robotPose.getY();

        // ---- Desired robot heading so the CAMERA faces the hub ----
        // atan2(dy, dx) is the field angle toward the hub from the robot.
        // Subtract the camera's yaw offset so the camera barrel (not robot nose)
        // points at the hub. kCameraYawDegrees is positive = camera rotated left
        // relative to robot, so robot must turn right by that much to compensate.
        double hubAngleDeg    = Math.toDegrees(Math.atan2(dy, dx));
        double desiredHeadingDeg = hubAngleDeg - VisionConstants.kCameraYawDegrees;

        // ---- Current robot heading ----
        double currentHeadingDeg = m_driveSubsystem.getHeading().getDegrees();

        // ---- PID output → rotation speed ----
        double rawRotationDeg = m_headingPID.calculate(currentHeadingDeg, desiredHeadingDeg);
        double rotationRadPerSec = MathUtil.clamp(
                Math.toRadians(rawRotationDeg),
                -OrbitalConstants.kOrbitalMaxRotationRadPerSec,
                 OrbitalConstants.kOrbitalMaxRotationRadPerSec);

        // Suppress tiny corrections when already on target
        if (m_headingPID.atSetpoint()) {
            rotationRadPerSec = 0.0;
        }

        // ---- Pass driver translation straight through (field-centric) ----
        // Throttle remaps [0,1] → [kThrottleMinFraction, 1.0] matching TeleopDriveCommand.
        double trigger = MathUtil.clamp(m_throttle.getAsDouble(), 0.0, 1.0);
        double min = SwerveConstants.kThrottleMinFraction;
        double speedMultiplier = min + (1.0 - min) * trigger;

        double xInput = MathUtil.applyDeadband(m_xSpeed.getAsDouble(), SwerveConstants.kDeadband);
        double yInput = MathUtil.applyDeadband(m_ySpeed.getAsDouble(), SwerveConstants.kDeadband);

        double vxMPS = xInput * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed * speedMultiplier;
        double vyMPS = yInput * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed * speedMultiplier;

        // ---- Drive: full driver translation + hub-aimed rotation ----
        m_driveSubsystem.drive(vxMPS, vyMPS, rotationRadPerSec, true);

        // ---- Telemetry (always on for this command) ----
        SmartDashboard.putNumber("Orbital/DesiredHeadingDeg",  desiredHeadingDeg);
        SmartDashboard.putNumber("Orbital/CurrentHeadingDeg",  currentHeadingDeg);
        SmartDashboard.putNumber("Orbital/HeadingErrorDeg",    desiredHeadingDeg - currentHeadingDeg);
        SmartDashboard.putBoolean("Orbital/AtSetpoint",        m_headingPID.atSetpoint());
        SmartDashboard.putNumber("Orbital/HubX",               hub.getX());
        SmartDashboard.putNumber("Orbital/HubY",               hub.getY());
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button is released
    }
}
