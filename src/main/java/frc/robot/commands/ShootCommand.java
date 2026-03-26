package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterTableConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ShooterInterpolation;

/**
 * Orchestrates the full shooting sequence with distance-based RPM adjustment.
 *
 * <ol>
 *   <li>At {@code initialize()}: reads the direct Limelight trig distance to the hub
 *       (tags 9/10 for Red, 25/26 for Blue) and interpolates shooter RPM from the lookup table.
 *       Falls back to {@link frc.robot.Constants.ShooterTableConstants#kFallbackShooterRPM}
 *       if no hub tags are visible.
 *   <li>Spins up flywheels to the interpolated RPM while running kicker.
 *   <li>Once both flywheels reach speed (or the spin-up timeout elapses), runs the agitator
 *       to feed the ball.
 *   <li>Stops everything when the command ends (button released).
 * </ol>
 *
 * <p>Runs until cancelled (e.g., button released).
 */
public class ShootCommand extends Command {

    private final ShooterSubsystem m_shooterSubsystem;
    private final VisionSubsystem  m_visionSubsystem;

    private final Timer m_spinUpTimer = new Timer();

    /** RPMs determined at initialize() and held constant for the duration of the shot. */
    private double m_targetShooterRPM;
    private double m_targetPreShooterRPM;

    /**
     * Creates a ShootCommand that adjusts RPM based on direct Limelight tag distance.
     *
     * @param shooterSubsystem The shooter subsystem.
     * @param visionSubsystem  The vision subsystem (used for direct tag distance).
     */
    public ShootCommand(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        m_visionSubsystem  = visionSubsystem;
        addRequirements(m_shooterSubsystem);
        // VisionSubsystem is read-only here — no requirement needed
    }

    @Override
    public void initialize() {
        m_spinUpTimer.restart();

        // Use ONLY the direct Limelight trig distance (tags 9/10 for Red, 25/26 for Blue).
        // The odometry-based distance is intentionally NOT used — pose-estimator drift can
        // make it read ~13 m when the robot is actually 2–3 m from the hub.
        double directDistance = m_visionSubsystem.getDirectDistanceToHub();
        String distanceSource;

        if (directDistance > 0) {
            m_targetShooterRPM = ShooterInterpolation.getShooterRPM(directDistance);
            distanceSource     = "Direct Tag";
        } else {
            m_targetShooterRPM = ShooterTableConstants.kFallbackShooterRPM;
            distanceSource     = "No Tag – Fallback RPM";
        }
        // Pre-shooter always runs at full speed regardless of distance
        m_targetPreShooterRPM = ShooterConstants.kPreShooterForwardRPM;

        SmartDashboard.putNumber("Shooter/Distance At Shot",      directDistance);
        SmartDashboard.putString("Shooter/Distance Source",       distanceSource);
        SmartDashboard.putNumber("Shooter/Target Shooter RPM",    m_targetShooterRPM);
        SmartDashboard.putNumber("Shooter/Target PreShooter RPM", m_targetPreShooterRPM);

        m_shooterSubsystem.runShooterAtRPM(m_targetShooterRPM);
        m_shooterSubsystem.runPreShooterAtRPM(m_targetPreShooterRPM);
        // Hold kicker in slow reverse during spin-up to prevent pre-loading the ball
        m_shooterSubsystem.runKickerSlowReverse();
    }

    @Override
    public void execute() {
        m_shooterSubsystem.runShooterAtRPM(m_targetShooterRPM);
        m_shooterSubsystem.runPreShooterAtRPM(m_targetPreShooterRPM);

        boolean atSpeed  = m_shooterSubsystem.isShooterAtSpeed(m_targetShooterRPM);
        boolean timedOut = m_spinUpTimer.hasElapsed(ShooterConstants.kShooterSpinUpTimeoutSeconds);

        SmartDashboard.putBoolean("Shooter/FeedGate AtSpeed",  atSpeed);
        SmartDashboard.putBoolean("Shooter/FeedGate TimedOut", timedOut);

        if (atSpeed || timedOut) {
            // Flywheels ready — run kicker forward to feed
            m_shooterSubsystem.runKicker();
            // Only run the agitator once the kicker is actually spinning up to feeding speed,
            // so the ball isn't pushed in before the kicker can grip and carry it
            if (m_shooterSubsystem.isKickerFeeding()) {
                m_shooterSubsystem.runAgitator();
            }
        } else {
            // Still spinning up — hold kicker in slow reverse to prevent pre-loading
            m_shooterSubsystem.runKickerSlowReverse();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
