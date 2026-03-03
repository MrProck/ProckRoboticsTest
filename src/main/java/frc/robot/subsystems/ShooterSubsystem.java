package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

/**
 * Shooter subsystem with a 4-stage shooting pipeline:
 *  - Agitator    (NEO + SparkMax, CAN 19, 20A)
 *  - Kicker      (NEO + SparkMax, CAN 20, 40A)
 *  - Pre-Shooter (NEO Vortex + SparkFlex, CAN 21, 40A)
 *  - Shooter     (2x NEO Vortex + SparkFlex, CAN 22 + 23, 140A each)
 */
public class ShooterSubsystem extends SubsystemBase {

    // Motors
    private final CANSparkMax  m_agitatorMotor          = new CANSparkMax(ShooterConstants.kAgitatorMotorID,         MotorType.kBrushless);
    private final CANSparkMax  m_kickerMotor             = new CANSparkMax(ShooterConstants.kKickerMotorID,           MotorType.kBrushless);
    private final CANSparkFlex m_preShooterMotor         = new CANSparkFlex(ShooterConstants.kPreShooterMotorID,      MotorType.kBrushless);
    private final CANSparkFlex m_shooterPrimaryMotor     = new CANSparkFlex(ShooterConstants.kShooterPrimaryMotorID,  MotorType.kBrushless);
    private final CANSparkFlex m_shooterSecondaryMotor   = new CANSparkFlex(ShooterConstants.kShooterSecondaryMotorID, MotorType.kBrushless);

    // PID controllers
    private final SparkPIDController m_agitatorPID;
    private final SparkPIDController m_kickerPID;
    private final SparkPIDController m_preShooterPID;
    private final SparkPIDController m_shooterPrimaryPID;
    private final SparkPIDController m_shooterSecondaryPID;

    public ShooterSubsystem() {
        // --- Agitator Motor (NEO + SparkMax) ---
        checkREV("Agitator restoreFactoryDefaults", m_agitatorMotor.restoreFactoryDefaults());
        checkREV("Agitator setIdleMode", m_agitatorMotor.setIdleMode(IdleMode.kBrake));
        checkREV("Agitator setSmartCurrentLimit", m_agitatorMotor.setSmartCurrentLimit(ShooterConstants.kAgitatorCurrentLimitAmps));
        m_agitatorMotor.setInverted(ShooterConstants.kAgitatorInverted);
        m_agitatorPID = m_agitatorMotor.getPIDController();
        checkREV("Agitator PID P", m_agitatorPID.setP(ShooterConstants.kAgitatorP));
        checkREV("Agitator PID I", m_agitatorPID.setI(ShooterConstants.kAgitatorI));
        checkREV("Agitator PID D", m_agitatorPID.setD(ShooterConstants.kAgitatorD));
        checkREV("Agitator PID FF", m_agitatorPID.setFF(ShooterConstants.kAgitatorFF));
        burnFlashWithDelay(m_agitatorMotor, "Agitator burnFlash");

        // --- Kicker Motor (NEO + SparkMax) ---
        checkREV("Kicker restoreFactoryDefaults", m_kickerMotor.restoreFactoryDefaults());
        checkREV("Kicker setIdleMode", m_kickerMotor.setIdleMode(IdleMode.kBrake));
        checkREV("Kicker setSmartCurrentLimit", m_kickerMotor.setSmartCurrentLimit(ShooterConstants.kKickerCurrentLimitAmps));
        m_kickerMotor.setInverted(ShooterConstants.kKickerInverted);
        m_kickerPID = m_kickerMotor.getPIDController();
        checkREV("Kicker PID P", m_kickerPID.setP(ShooterConstants.kKickerP));
        checkREV("Kicker PID I", m_kickerPID.setI(ShooterConstants.kKickerI));
        checkREV("Kicker PID D", m_kickerPID.setD(ShooterConstants.kKickerD));
        checkREV("Kicker PID FF", m_kickerPID.setFF(ShooterConstants.kKickerFF));
        burnFlashWithDelay(m_kickerMotor, "Kicker burnFlash");

        // --- Pre-Shooter Motor (NEO Vortex + SparkFlex) ---
        checkREV("PreShooter restoreFactoryDefaults", m_preShooterMotor.restoreFactoryDefaults());
        checkREV("PreShooter setIdleMode", m_preShooterMotor.setIdleMode(IdleMode.kCoast));
        checkREV("PreShooter setSmartCurrentLimit", m_preShooterMotor.setSmartCurrentLimit(ShooterConstants.kPreShooterCurrentLimitAmps));
        m_preShooterMotor.setInverted(ShooterConstants.kPreShooterInverted);
        m_preShooterPID = m_preShooterMotor.getPIDController();
        checkREV("PreShooter PID P", m_preShooterPID.setP(ShooterConstants.kPreShooterP));
        checkREV("PreShooter PID I", m_preShooterPID.setI(ShooterConstants.kPreShooterI));
        checkREV("PreShooter PID D", m_preShooterPID.setD(ShooterConstants.kPreShooterD));
        checkREV("PreShooter PID FF", m_preShooterPID.setFF(ShooterConstants.kPreShooterFF));
        burnFlashWithDelay(m_preShooterMotor, "PreShooter burnFlash");

        // --- Shooter Primary Motor (NEO Vortex + SparkFlex, CAN 22) ---
        checkREV("ShooterPrimary restoreFactoryDefaults", m_shooterPrimaryMotor.restoreFactoryDefaults());
        checkREV("ShooterPrimary setIdleMode", m_shooterPrimaryMotor.setIdleMode(IdleMode.kCoast));
        checkREV("ShooterPrimary setSmartCurrentLimit", m_shooterPrimaryMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimitAmps));
        m_shooterPrimaryMotor.setInverted(ShooterConstants.kShooterPrimaryInverted);
        m_shooterPrimaryPID = m_shooterPrimaryMotor.getPIDController();
        checkREV("ShooterPrimary PID P", m_shooterPrimaryPID.setP(ShooterConstants.kShooterP));
        checkREV("ShooterPrimary PID I", m_shooterPrimaryPID.setI(ShooterConstants.kShooterI));
        checkREV("ShooterPrimary PID D", m_shooterPrimaryPID.setD(ShooterConstants.kShooterD));
        checkREV("ShooterPrimary PID FF", m_shooterPrimaryPID.setFF(ShooterConstants.kShooterFF));
        burnFlashWithDelay(m_shooterPrimaryMotor, "ShooterPrimary burnFlash");

        // --- Shooter Secondary Motor (NEO Vortex + SparkFlex, CAN 23) ---
        checkREV("ShooterSecondary restoreFactoryDefaults", m_shooterSecondaryMotor.restoreFactoryDefaults());
        checkREV("ShooterSecondary setIdleMode", m_shooterSecondaryMotor.setIdleMode(IdleMode.kCoast));
        checkREV("ShooterSecondary setSmartCurrentLimit", m_shooterSecondaryMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimitAmps));
        m_shooterSecondaryMotor.setInverted(ShooterConstants.kShooterSecondaryInverted);
        m_shooterSecondaryPID = m_shooterSecondaryMotor.getPIDController();
        checkREV("ShooterSecondary PID P", m_shooterSecondaryPID.setP(ShooterConstants.kShooterP));
        checkREV("ShooterSecondary PID I", m_shooterSecondaryPID.setI(ShooterConstants.kShooterI));
        checkREV("ShooterSecondary PID D", m_shooterSecondaryPID.setD(ShooterConstants.kShooterD));
        checkREV("ShooterSecondary PID FF", m_shooterSecondaryPID.setFF(ShooterConstants.kShooterFF));
        burnFlashWithDelay(m_shooterSecondaryMotor, "ShooterSecondary burnFlash");
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

    /**
     * Waits 200 ms (REV-recommended delay) then burns flash, logging any error.
     *
     * @param motor  The motor whose flash to burn
     * @param label  A human-readable label for logging
     */
    private static void burnFlashWithDelay(CANSparkBase motor, String label) {
        try { Thread.sleep(200); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        checkREV(label, motor.burnFlash());
    }

    // -------------------------------------------------------------------------
    // Agitator
    // -------------------------------------------------------------------------

    /** Runs the agitator motor at the configured forward RPM using closed-loop velocity control. */
    public void runAgitator() {
        m_agitatorPID.setReference(ShooterConstants.kAgitatorForwardRPM, ControlType.kVelocity);
    }

    /** Stops the agitator motor. */
    public void stopAgitator() {
        m_agitatorMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Kicker
    // -------------------------------------------------------------------------

    /** Runs the kicker motor at the configured forward RPM using closed-loop velocity control. */
    public void runKicker() {
        m_kickerPID.setReference(ShooterConstants.kKickerForwardRPM, ControlType.kVelocity);
    }

    /** Stops the kicker motor. */
    public void stopKicker() {
        m_kickerMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Pre-Shooter
    // -------------------------------------------------------------------------

    /** Runs the pre-shooter motor at the configured forward RPM using closed-loop velocity control. */
    public void runPreShooter() {
        m_preShooterPID.setReference(ShooterConstants.kPreShooterForwardRPM, ControlType.kVelocity);
    }

    /** Stops the pre-shooter motor. */
    public void stopPreShooter() {
        m_preShooterMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Shooter
    // -------------------------------------------------------------------------

    /** Runs both shooter flywheel motors at the configured forward RPM using closed-loop velocity control. */
    public void runShooter() {
        m_shooterPrimaryPID.setReference(ShooterConstants.kShooterForwardRPM, ControlType.kVelocity);
        m_shooterSecondaryPID.setReference(ShooterConstants.kShooterForwardRPM, ControlType.kVelocity);
    }

    /** Stops both shooter flywheel motors. */
    public void stopShooter() {
        m_shooterPrimaryMotor.stopMotor();
        m_shooterSecondaryMotor.stopMotor();
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
        m_agitatorPID.setReference(-ShooterConstants.kAgitatorReverseRPM, ControlType.kVelocity);
        m_kickerPID.setReference(-ShooterConstants.kKickerReverseRPM, ControlType.kVelocity);
        m_preShooterPID.setReference(-ShooterConstants.kPreShooterReverseRPM, ControlType.kVelocity);
        m_shooterPrimaryPID.setReference(-ShooterConstants.kShooterReverseRPM, ControlType.kVelocity);
        m_shooterSecondaryPID.setReference(-ShooterConstants.kShooterReverseRPM, ControlType.kVelocity);
    }

    /** Stops all 4 stages (5 motors). */
    public void stopAll() {
        stopAgitator();
        stopKicker();
        stopPreShooter();
        stopShooter();
    }

    /**
     * Returns true if both shooter flywheel motors are within tolerance of the target RPM.
     * Useful for commands that need to wait for spin-up before feeding.
     */
    public boolean isShooterAtSpeed() {
        double primaryVelocity = m_shooterPrimaryMotor.getEncoder().getVelocity();
        double secondaryVelocity = m_shooterSecondaryMotor.getEncoder().getVelocity();
        return Math.abs(primaryVelocity - ShooterConstants.kShooterTargetRPM) <= ShooterConstants.kShooterSpeedToleranceRPM
            && Math.abs(secondaryVelocity - ShooterConstants.kShooterTargetRPM) <= ShooterConstants.kShooterSpeedToleranceRPM;
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Agitator RPM",          m_agitatorMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Kicker RPM",            m_kickerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/PreShooter RPM",        m_preShooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Shooter Primary RPM",   m_shooterPrimaryMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Shooter Secondary RPM", m_shooterSecondaryMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter/AtSpeed",              isShooterAtSpeed());
    }
}
