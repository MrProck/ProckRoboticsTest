package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    private boolean m_feeding    = false;
    private boolean m_rpmLocked  = false;  // true once a valid tag distance has been used to set RPM

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
        m_feeding   = false;
        m_rpmLocked = false;

        // Start with fallback RPM so flywheels begin spinning immediately.
        // The RPM will be updated in execute() as soon as a valid tag distance is seen.
        m_targetShooterRPM    = ShooterTableConstants.kFallbackShooterRPM;
        m_targetPreShooterRPM = ShooterConstants.kPreShooterForwardRPM;

        SmartDashboard.putString("AutoShoot/Distance Source",      "Waiting for tag...");
        SmartDashboard.putNumber("AutoShoot/Distance At Shot",     -1.0);
        SmartDashboard.putNumber("AutoShoot/Direct Distance",      -1.0);
        SmartDashboard.putNumber("AutoShoot/Target Shooter RPM",    m_targetShooterRPM);
        SmartDashboard.putNumber("AutoShoot/Target PreShooter RPM", m_targetPreShooterRPM);

        // Start spinning up immediately; hold kicker in slow reverse to prevent pre-loading
        m_shooterSubsystem.runShooterAtRPM(m_targetShooterRPM);
        m_shooterSubsystem.runPreShooterAtRPM(m_targetPreShooterRPM);
        m_shooterSubsystem.runKickerSlowReverse();
    }

    @Override
    public void execute() {
        // --- Distance-based RPM update ---
        // Keep re-checking for a valid tag distance until one is found and locked in.
        // This handles the common auto case where the robot arrives at the shoot position
        // before the Limelight has acquired the hub tag.
        if (!m_rpmLocked) {
            double directDistance = m_visionSubsystem.getDirectDistanceToHub();
            if (directDistance > 0) {
                m_targetShooterRPM = ShooterInterpolation.getShooterRPM(directDistance);
                m_rpmLocked = true;
                m_spinUpTimer.restart();  // reset timeout so motors have a full 3s to reach the correct RPM
                SmartDashboard.putString("AutoShoot/Distance Source",      "Direct Tag");
                SmartDashboard.putNumber("AutoShoot/Distance At Shot",     directDistance);
                SmartDashboard.putNumber("AutoShoot/Direct Distance",      directDistance);
                SmartDashboard.putNumber("AutoShoot/Target Shooter RPM",   m_targetShooterRPM);
            } else {
                SmartDashboard.putString("AutoShoot/Distance Source",      "No Tag – Fallback RPM");
                SmartDashboard.putNumber("AutoShoot/Direct Distance",      -1.0);
            }
        }

        // --- Hub-facing rotation (same logic as OrbitalDriveCommand) ---
        Translation2d hub = m_visionSubsystem.getHubTranslation();
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
            // Flywheels ready — run kicker forward to feed
            m_shooterSubsystem.runKicker();
            // Only run the agitator once the kicker is actually spinning up to feeding speed,
            // so the ball isn't pushed in before the kicker can grip and carry it
            if (m_shooterSubsystem.isKickerFeeding()) {
                m_shooterSubsystem.runAgitator();
                if (!m_feeding) {
                    m_feedTimer.restart();
                    m_feeding = true;
                }
            }
        } else {
            // Still spinning up — hold kicker in slow reverse to prevent pre-loading
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

}
