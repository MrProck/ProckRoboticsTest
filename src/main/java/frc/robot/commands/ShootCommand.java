package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Orchestrates the full shooting sequence:
 *  1. Spins up the pre-shooter and shooter flywheels immediately.
 *  2. Once the shooter is at speed (or the spin-up timeout elapses), starts
 *     the agitator and kicker to feed the game piece into the spinning flywheels.
 *  3. Stops everything when the command ends.
 *
 * Runs until cancelled (e.g., button released).
 */
public class ShootCommand extends Command {

    private final ShooterSubsystem m_shooterSubsystem;
    private boolean m_feedingStarted;
    private final Timer m_spinUpTimer = new Timer();

    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_feedingStarted = false;
        m_spinUpTimer.restart();
        m_shooterSubsystem.runPreShooter();
        m_shooterSubsystem.runShooter();
    }

    @Override
    public void execute() {
        // Continuously command flywheels to maintain speed
        m_shooterSubsystem.runPreShooter();
        m_shooterSubsystem.runShooter();

        if (!m_feedingStarted) {
            boolean atSpeed = m_shooterSubsystem.isShooterAtSpeed();
            boolean timedOut = m_spinUpTimer.hasElapsed(ShooterConstants.kShooterSpinUpTimeoutSeconds);

            if (atSpeed || timedOut) {
                if (timedOut && !atSpeed) {
                    System.err.println("[ShootCommand] WARNING: Spin-up timeout elapsed — feeding without reaching target RPM");
                }
                m_shooterSubsystem.runAgitator();
                m_shooterSubsystem.runKicker();
                m_feedingStarted = true;
            }
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
