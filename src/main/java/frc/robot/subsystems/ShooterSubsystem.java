package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.util.REVUtil;

/**
 * Shooter subsystem with a 4-stage shooting pipeline:
 *  - Agitator    (NEO + SparkMax, CAN 19, 20A)
 *  - Kicker      (NEO Vortex + SparkFlex, CAN 20, 40A)
 *  - Pre-Shooter (NEO Vortex + SparkFlex, CAN 21, 40A)
 *  - Shooter     (2x NEO Vortex + SparkFlex, CAN 22 + 23, 80A each)
 */
public class ShooterSubsystem extends SubsystemBase {

    // Motors
    private final CANSparkMax  m_agitatorMotor          = new CANSparkMax(ShooterConstants.kAgitatorMotorID,         MotorType.kBrushless);
    private final CANSparkFlex m_kickerMotor             = new CANSparkFlex(ShooterConstants.kKickerMotorID,          MotorType.kBrushless);
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
        REVUtil.checkREV("Agitator restoreFactoryDefaults", m_agitatorMotor.restoreFactoryDefaults());
        REVUtil.checkREV("Agitator setIdleMode", m_agitatorMotor.setIdleMode(IdleMode.kBrake));
        REVUtil.checkREV("Agitator setSmartCurrentLimit", m_agitatorMotor.setSmartCurrentLimit(ShooterConstants.kAgitatorCurrentLimitAmps));
        m_agitatorMotor.setInverted(ShooterConstants.kAgitatorInverted);
        m_agitatorPID = m_agitatorMotor.getPIDController();
        REVUtil.checkREV("Agitator PID P", m_agitatorPID.setP(ShooterConstants.kAgitatorP));
        REVUtil.checkREV("Agitator PID I", m_agitatorPID.setI(ShooterConstants.kAgitatorI));
        REVUtil.checkREV("Agitator PID D", m_agitatorPID.setD(ShooterConstants.kAgitatorD));
        REVUtil.checkREV("Agitator PID FF", m_agitatorPID.setFF(ShooterConstants.kAgitatorFF));
        REVUtil.burnFlashWithDelay(m_agitatorMotor, "Agitator burnFlash");

        // --- Kicker Motor (NEO Vortex + SparkFlex) ---
        REVUtil.checkREV("Kicker restoreFactoryDefaults", m_kickerMotor.restoreFactoryDefaults());
        REVUtil.checkREV("Kicker setIdleMode", m_kickerMotor.setIdleMode(IdleMode.kBrake));
        REVUtil.checkREV("Kicker setSmartCurrentLimit", m_kickerMotor.setSmartCurrentLimit(ShooterConstants.kKickerCurrentLimitAmps));
        m_kickerMotor.setInverted(ShooterConstants.kKickerInverted);
        m_kickerPID = m_kickerMotor.getPIDController();
        REVUtil.checkREV("Kicker PID P", m_kickerPID.setP(ShooterConstants.kKickerP));
        REVUtil.checkREV("Kicker PID I", m_kickerPID.setI(ShooterConstants.kKickerI));
        REVUtil.checkREV("Kicker PID D", m_kickerPID.setD(ShooterConstants.kKickerD));
        REVUtil.checkREV("Kicker PID FF", m_kickerPID.setFF(ShooterConstants.kKickerFF));
        REVUtil.burnFlashWithDelay(m_kickerMotor, "Kicker burnFlash");

        // --- Pre-Shooter Motor (NEO Vortex + SparkFlex) ---
        REVUtil.checkREV("PreShooter restoreFactoryDefaults", m_preShooterMotor.restoreFactoryDefaults());
        REVUtil.checkREV("PreShooter setIdleMode", m_preShooterMotor.setIdleMode(IdleMode.kCoast));
        REVUtil.checkREV("PreShooter setSmartCurrentLimit", m_preShooterMotor.setSmartCurrentLimit(ShooterConstants.kPreShooterCurrentLimitAmps));
        m_preShooterMotor.setInverted(ShooterConstants.kPreShooterInverted);
        m_preShooterPID = m_preShooterMotor.getPIDController();
        REVUtil.checkREV("PreShooter PID P", m_preShooterPID.setP(ShooterConstants.kPreShooterP));
        REVUtil.checkREV("PreShooter PID I", m_preShooterPID.setI(ShooterConstants.kPreShooterI));
        REVUtil.checkREV("PreShooter PID D", m_preShooterPID.setD(ShooterConstants.kPreShooterD));
        REVUtil.checkREV("PreShooter PID FF", m_preShooterPID.setFF(ShooterConstants.kPreShooterFF));
        REVUtil.burnFlashWithDelay(m_preShooterMotor, "PreShooter burnFlash");

        // --- Shooter Primary Motor (NEO Vortex + SparkFlex, CAN 22) ---
        REVUtil.checkREV("ShooterPrimary restoreFactoryDefaults", m_shooterPrimaryMotor.restoreFactoryDefaults());
        REVUtil.checkREV("ShooterPrimary setIdleMode", m_shooterPrimaryMotor.setIdleMode(IdleMode.kCoast));
        REVUtil.checkREV("ShooterPrimary setSmartCurrentLimit", m_shooterPrimaryMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimitAmps));
        m_shooterPrimaryMotor.setInverted(ShooterConstants.kShooterPrimaryInverted);
        m_shooterPrimaryPID = m_shooterPrimaryMotor.getPIDController();
        REVUtil.checkREV("ShooterPrimary PID P", m_shooterPrimaryPID.setP(ShooterConstants.kShooterP));
        REVUtil.checkREV("ShooterPrimary PID I", m_shooterPrimaryPID.setI(ShooterConstants.kShooterI));
        REVUtil.checkREV("ShooterPrimary PID D", m_shooterPrimaryPID.setD(ShooterConstants.kShooterD));
        REVUtil.checkREV("ShooterPrimary PID FF", m_shooterPrimaryPID.setFF(ShooterConstants.kShooterFF));
        REVUtil.burnFlashWithDelay(m_shooterPrimaryMotor, "ShooterPrimary burnFlash");

        // --- Shooter Secondary Motor (NEO Vortex + SparkFlex, CAN 23) ---
        REVUtil.checkREV("ShooterSecondary restoreFactoryDefaults", m_shooterSecondaryMotor.restoreFactoryDefaults());
        REVUtil.checkREV("ShooterSecondary setIdleMode", m_shooterSecondaryMotor.setIdleMode(IdleMode.kCoast));
        REVUtil.checkREV("ShooterSecondary setSmartCurrentLimit", m_shooterSecondaryMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimitAmps));
        m_shooterSecondaryMotor.setInverted(ShooterConstants.kShooterSecondaryInverted);
        m_shooterSecondaryPID = m_shooterSecondaryMotor.getPIDController();
        REVUtil.checkREV("ShooterSecondary PID P", m_shooterSecondaryPID.setP(ShooterConstants.kShooterP));
        REVUtil.checkREV("ShooterSecondary PID I", m_shooterSecondaryPID.setI(ShooterConstants.kShooterI));
        REVUtil.checkREV("ShooterSecondary PID D", m_shooterSecondaryPID.setD(ShooterConstants.kShooterD));
        REVUtil.checkREV("ShooterSecondary PID FF", m_shooterSecondaryPID.setFF(ShooterConstants.kShooterFF));
        REVUtil.burnFlashWithDelay(m_shooterSecondaryMotor, "ShooterSecondary burnFlash");
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
