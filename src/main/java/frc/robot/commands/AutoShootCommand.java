package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OrbitalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterTableConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ShooterInterpolation;

import java.util.Optional;

/**
 * Autonomous shoot command that rotates the robot to face the alliance hub
 * while simultaneously spinning up the shooter flywheels, then feeds once
 * both the heading and flywheel speed conditions are met.
 *
 * <p>Requires both the DriveSubsystem and ShooterSubsystem. Uses the same
 * hub-facing PID gains as OrbitalDriveCommand (OrbitalConstants.kOrbitalP/D).
 *
 * <p>Finishes when the shot is fired (after feeding begins), so PathPlanner
 * can sequence the next action. Specifically, it finishes one loop cycle after
 * the feed gate opens (atSpeed or timedOut).
 */
public class AutoShootCommand extends Command {

    private final DriveSubsystem   m_driveSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final VisionSubsystem  m_visionSubsystem;

    private final PIDController m_headingPID;
    private final Timer         m_spinUpTimer = new Timer();
    private final Timer         m_feedTimer   = new Timer();

    private double  m_targetShooterRPM;
    private double  m_targetPreShooterRPM;
    private boolean m_feeding = false;

    public AutoShootCommand(
            DriveSubsystem driveSubsystem,
            ShooterSubsystem shooterSubsystem,
            VisionSubsystem visionSubsystem) {
        m_driveSubsystem   = driveSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_visionSubsystem  = visionSubsystem;

        m_headingPID = new PIDController(
                OrbitalConstants.kOrbitalP,
                OrbitalConstants.kOrbitalI,
                OrbitalConstants.kOrbitalD);

        // Continuous input so the PID wraps correctly across the ±180° boundary
        m_headingPID.enableContinuousInput(-180.0, 180.0);
        m_headingPID.setTolerance(OrbitalConstants.kOrbitalToleranceDegrees);

        addRequirements(m_driveSubsystem, m_shooterSubsystem);
        // VisionSubsystem is read-only — no requirement
    }

    @Override
    public void initialize() {
        m_headingPID.reset();
        m_spinUpTimer.restart();
        m_feedTimer.stop();
        m_feedTimer.reset();
        m_feeding = false;

        // Lock in RPM at the moment we start (distance from odometry)
        double distanceMeters = m_visionSubsystem.getDistanceToHub();
        if (distanceMeters > 0) {
            m_targetShooterRPM    = ShooterInterpolation.getShooterRPM(distanceMeters);
        } else {
            m_targetShooterRPM    = ShooterTableConstants.kFallbackShooterRPM;
        }
        // Pre-shooter always runs at full speed regardless of distance
        m_targetPreShooterRPM = ShooterConstants.kPreShooterForwardRPM;

        SmartDashboard.putNumber("AutoShoot/Distance At Shot",      distanceMeters);
        SmartDashboard.putNumber("AutoShoot/Target Shooter RPM",    m_targetShooterRPM);
        SmartDashboard.putNumber("AutoShoot/Target PreShooter RPM", m_targetPreShooterRPM);

        // Start spinning up immediately
        m_shooterSubsystem.runShooterAtRPM(m_targetShooterRPM);
        m_shooterSubsystem.runPreShooterAtRPM(m_targetPreShooterRPM);
        m_shooterSubsystem.runKickerSlowReverse();
    }

    @Override
    public void execute() {
        // --- Hub-facing rotation (same logic as OrbitalDriveCommand) ---
        Translation2d hub = getHubPosition();
        Pose2d robotPose = m_driveSubsystem.getPose();

        // Vector from hub to robot (same convention as OrbitalDriveCommand)
        double dx = robotPose.getX() - hub.getX();
        double dy = robotPose.getY() - hub.getY();

        // Negate to get the direction from robot toward hub
        double desiredHeadingDeg = Math.toDegrees(Math.atan2(-dy, -dx));
        double currentHeadingDeg = m_driveSubsystem.getHeading().getDegrees();

        // PID operates in degrees; output is proportional to degree-error
        double rawRotationDeg = m_headingPID.calculate(currentHeadingDeg, desiredHeadingDeg);
        double omega = MathUtil.clamp(
                Math.toRadians(rawRotationDeg),
                -OrbitalConstants.kOrbitalMaxRotationRadPerSec,
                 OrbitalConstants.kOrbitalMaxRotationRadPerSec);

        // Suppress tiny corrections when already on target
        if (m_headingPID.atSetpoint()) {
            omega = 0.0;
        }

        // Drive: no translation, only rotation to face hub
        m_driveSubsystem.drive(0.0, 0.0, omega, true);

        boolean atHeading = m_headingPID.atSetpoint();
        SmartDashboard.putBoolean("AutoShoot/AtHeading", atHeading);
        SmartDashboard.putNumber("AutoShoot/HeadingError (deg)",
                desiredHeadingDeg - currentHeadingDeg);

        // --- Shooter spin-up ---
        m_shooterSubsystem.runShooterAtRPM(m_targetShooterRPM);
        m_shooterSubsystem.runPreShooterAtRPM(m_targetPreShooterRPM);

        boolean atSpeed  = m_shooterSubsystem.isShooterAtSpeed(m_targetShooterRPM);
        boolean timedOut = m_spinUpTimer.hasElapsed(ShooterConstants.kShooterSpinUpTimeoutSeconds);

        SmartDashboard.putBoolean("AutoShoot/FeedGate AtSpeed",  atSpeed);
        SmartDashboard.putBoolean("AutoShoot/FeedGate TimedOut", timedOut);

        // Feed only when aimed at hub AND shooter at speed (or timed out as fallback)
        if ((atHeading || timedOut) && (atSpeed || timedOut)) {
            m_shooterSubsystem.runAgitator();
            m_shooterSubsystem.runKicker();
            if (!m_feeding) {
                m_feedTimer.restart();
                m_feeding = true;
            }
        } else {
            m_shooterSubsystem.runKickerSlowReverse();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stopAll();
        m_driveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // Finish after feeding for the minimum feed duration so the ball has time to leave
        return m_feeding && m_feedTimer.hasElapsed(ShooterConstants.kAutoFeedDurationSeconds);
    }

    /** Returns the hub position for the current alliance. Defaults to Blue if unknown. */
    private Translation2d getHubPosition() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return new Translation2d(OrbitalConstants.kHubRedX, OrbitalConstants.kHubY);
        }
        return new Translation2d(OrbitalConstants.kHubBlueX, OrbitalConstants.kHubY);
    }
}
