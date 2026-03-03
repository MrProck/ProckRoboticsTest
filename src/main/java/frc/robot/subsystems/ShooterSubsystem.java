package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

/**
 * Shooter subsystem with a 4-stage shooting pipeline:
 *  - Agitator    (NEO + SparkMax, CAN 19)
 *  - Kicker      (NEO + SparkMax, CAN 20)
 *  - Pre-Shooter (NEO + SparkMax, CAN 21)
 *  - Shooter     (NEO + SparkMax, CAN 22)
 */
public class ShooterSubsystem extends SubsystemBase {

    // Motors
    private final CANSparkMax m_agitatorMotor   = new CANSparkMax(ShooterConstants.kAgitatorMotorID,   MotorType.kBrushless);
    private final CANSparkMax m_kickerMotor      = new CANSparkMax(ShooterConstants.kKickerMotorID,     MotorType.kBrushless);
    private final CANSparkMax m_preShooterMotor  = new CANSparkMax(ShooterConstants.kPreShooterMotorID, MotorType.kBrushless);
    private final CANSparkMax m_shooterMotor     = new CANSparkMax(ShooterConstants.kShooterMotorID,    MotorType.kBrushless);

    public ShooterSubsystem() {
        // --- Agitator Motor ---
        checkREV("Agitator restoreFactoryDefaults", m_agitatorMotor.restoreFactoryDefaults());
        checkREV("Agitator setIdleMode", m_agitatorMotor.setIdleMode(IdleMode.kBrake));
        checkREV("Agitator setSmartCurrentLimit", m_agitatorMotor.setSmartCurrentLimit(ShooterConstants.kAgitatorCurrentLimitAmps));
        m_agitatorMotor.setInverted(ShooterConstants.kAgitatorInverted);
        checkREV("Agitator burnFlash", m_agitatorMotor.burnFlash());

        // --- Kicker Motor ---
        checkREV("Kicker restoreFactoryDefaults", m_kickerMotor.restoreFactoryDefaults());
        checkREV("Kicker setIdleMode", m_kickerMotor.setIdleMode(IdleMode.kBrake));
        checkREV("Kicker setSmartCurrentLimit", m_kickerMotor.setSmartCurrentLimit(ShooterConstants.kKickerCurrentLimitAmps));
        m_kickerMotor.setInverted(ShooterConstants.kKickerInverted);
        checkREV("Kicker burnFlash", m_kickerMotor.burnFlash());

        // --- Pre-Shooter Motor ---
        checkREV("PreShooter restoreFactoryDefaults", m_preShooterMotor.restoreFactoryDefaults());
        checkREV("PreShooter setIdleMode", m_preShooterMotor.setIdleMode(IdleMode.kCoast));
        checkREV("PreShooter setSmartCurrentLimit", m_preShooterMotor.setSmartCurrentLimit(ShooterConstants.kPreShooterCurrentLimitAmps));
        m_preShooterMotor.setInverted(ShooterConstants.kPreShooterInverted);
        checkREV("PreShooter burnFlash", m_preShooterMotor.burnFlash());

        // --- Shooter Motor ---
        checkREV("Shooter restoreFactoryDefaults", m_shooterMotor.restoreFactoryDefaults());
        checkREV("Shooter setIdleMode", m_shooterMotor.setIdleMode(IdleMode.kCoast));
        checkREV("Shooter setSmartCurrentLimit", m_shooterMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimitAmps));
        m_shooterMotor.setInverted(ShooterConstants.kShooterInverted);
        checkREV("Shooter burnFlash", m_shooterMotor.burnFlash());
    }

    /**
     * Checks a REVLibError and logs a warning if it is not OK.
     *
     * @param label   A human-readable label for the operation
     * @param error   The REVLibError returned by the REV API call
     */
    private static void checkREV(String label, REVLibError error) {
        if (error != REVLibError.kOk) {
            System.err.println("[ShooterSubsystem] " + label + " failed: " + error);
        }
    }

    // -------------------------------------------------------------------------
    // Agitator
    // -------------------------------------------------------------------------

    /** Runs the agitator motor at the configured speed. */
    public void runAgitator() {
        m_agitatorMotor.set(ShooterConstants.kAgitatorSpeed);
    }

    /** Stops the agitator motor. */
    public void stopAgitator() {
        m_agitatorMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Kicker
    // -------------------------------------------------------------------------

    /** Runs the kicker motor at the configured speed. */
    public void runKicker() {
        m_kickerMotor.set(ShooterConstants.kKickerSpeed);
    }

    /** Stops the kicker motor. */
    public void stopKicker() {
        m_kickerMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Pre-Shooter
    // -------------------------------------------------------------------------

    /** Runs the pre-shooter motor at the configured speed. */
    public void runPreShooter() {
        m_preShooterMotor.set(ShooterConstants.kPreShooterSpeed);
    }

    /** Stops the pre-shooter motor. */
    public void stopPreShooter() {
        m_preShooterMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Shooter
    // -------------------------------------------------------------------------

    /** Runs the shooter flywheel motor at the configured speed. */
    public void runShooter() {
        m_shooterMotor.set(ShooterConstants.kShooterSpeed);
    }

    /** Stops the shooter flywheel motor. */
    public void stopShooter() {
        m_shooterMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Combined
    // -------------------------------------------------------------------------

    /** Runs all 4 stages simultaneously. */
    public void runAll() {
        runAgitator();
        runKicker();
        runPreShooter();
        runShooter();
    }

    /** Runs all 4 stages simultaneously in reverse for clearing jams. */
    public void reverseAll() {
        m_agitatorMotor.set(-ShooterConstants.kAgitatorSpeed);
        m_kickerMotor.set(-ShooterConstants.kKickerSpeed);
        m_preShooterMotor.set(-ShooterConstants.kPreShooterSpeed);
        m_shooterMotor.set(-ShooterConstants.kShooterSpeed);
    }

    /** Stops all 4 motors. */
    public void stopAll() {
        stopAgitator();
        stopKicker();
        stopPreShooter();
        stopShooter();
    }

    /**
     * Returns true if the shooter flywheel is within tolerance of the target RPM.
     * Useful for commands that need to wait for spin-up before feeding.
     */
    public boolean isShooterAtSpeed() {
        double velocity = m_shooterMotor.getEncoder().getVelocity();
        return Math.abs(velocity - ShooterConstants.kShooterTargetRPM) <= ShooterConstants.kShooterSpeedToleranceRPM;
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Agitator Speed",        m_agitatorMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Kicker Speed",          m_kickerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/PreShooter Velocity RPM", m_preShooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Shooter Velocity RPM",  m_shooterMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter/AtSpeed",              isShooterAtSpeed());
    }
}
