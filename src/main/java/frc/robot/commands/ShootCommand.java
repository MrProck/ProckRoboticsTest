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
 *   <li>At {@code initialize()}: reads distance-to-hub from VisionSubsystem and interpolates
 *       shooter and pre-shooter RPMs from the lookup table in ShooterTableConstants.
 *       Falls back to the configured default RPMs if vision is unavailable (distance &lt;= 0).
 *   <li>Spins up flywheels to the interpolated RPM while running the kicker slowly in reverse
 *       to prevent pre-loading.
 *   <li>Once both flywheels reach speed (or the spin-up timeout elapses), runs the agitator
 *       and kicker in the forward direction to feed the ball.
 *   <li>Stops everything when the command ends.
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
     * Creates a ShootCommand that adjusts RPM based on distance from the hub.
     *
     * @param shooterSubsystem The shooter subsystem.
     * @param visionSubsystem  The vision subsystem (used for hub distance via odometry).
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

        double distanceMeters = m_visionSubsystem.getDistanceToHub();
        if (distanceMeters > 0) {
            m_targetShooterRPM    = ShooterInterpolation.getShooterRPM(distanceMeters);
        } else {
            m_targetShooterRPM    = ShooterTableConstants.kFallbackShooterRPM;
        }
        // Pre-shooter always runs at full speed regardless of distance
        m_targetPreShooterRPM = ShooterConstants.kPreShooterForwardRPM;

        SmartDashboard.putNumber("Shooter/Distance At Shot",      distanceMeters);
        SmartDashboard.putNumber("Shooter/Target Shooter RPM",    m_targetShooterRPM);
        SmartDashboard.putNumber("Shooter/Target PreShooter RPM", m_targetPreShooterRPM);

        m_shooterSubsystem.runShooterAtRPM(m_targetShooterRPM);
        m_shooterSubsystem.runPreShooterAtRPM(m_targetPreShooterRPM);
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
            m_shooterSubsystem.runAgitator();
            m_shooterSubsystem.runKicker();
        } else {
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
