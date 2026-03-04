package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

/**
 * Shooter subsystem with a 4-stage shooting pipeline:
 *  - Agitator    (NEO + SparkMax, CAN 19, 20A)
 *  - Kicker      (NEO Vortex + SparkFlex, CAN 20, 40A)
 *  - Pre-Shooter (NEO Vortex + SparkFlex, CAN 21, 40A)
 *  - Shooter     (2x NEO Vortex + SparkFlex, CAN 22 + 23, 80A each)
 */
public class ShooterSubsystem extends SubsystemBase {

    // Motors
    private final SparkMax  m_agitatorMotor          = new SparkMax(ShooterConstants.kAgitatorMotorID,         MotorType.kBrushless);
    private final SparkFlex m_kickerMotor             = new SparkFlex(ShooterConstants.kKickerMotorID,          MotorType.kBrushless);
    private final SparkFlex m_preShooterMotor         = new SparkFlex(ShooterConstants.kPreShooterMotorID,      MotorType.kBrushless);
    private final SparkFlex m_shooterPrimaryMotor     = new SparkFlex(ShooterConstants.kShooterPrimaryMotorID,  MotorType.kBrushless);
    private final SparkFlex m_shooterSecondaryMotor   = new SparkFlex(ShooterConstants.kShooterSecondaryMotorID, MotorType.kBrushless);

    // Closed-loop controllers
    private final SparkClosedLoopController m_agitatorController;
    private final SparkClosedLoopController m_kickerController;
    private final SparkClosedLoopController m_preShooterController;
    private final SparkClosedLoopController m_shooterPrimaryController;
    private final SparkClosedLoopController m_shooterSecondaryController;

    public ShooterSubsystem() {
        // --- Agitator Motor (NEO + SparkMax) ---
        SparkMaxConfig agitatorConfig = new SparkMaxConfig();
        agitatorConfig
            .inverted(ShooterConstants.kAgitatorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.kAgitatorCurrentLimitAmps);
        agitatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kAgitatorP)
            .i(ShooterConstants.kAgitatorI)
            .d(ShooterConstants.kAgitatorD)
            .feedForward
            .kV(ShooterConstants.kAgitatorFF * 12.0);
        m_agitatorMotor.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_agitatorController = m_agitatorMotor.getClosedLoopController();

        // --- Kicker Motor (NEO Vortex + SparkFlex) ---
        SparkFlexConfig kickerConfig = new SparkFlexConfig();
        kickerConfig
            .inverted(ShooterConstants.kKickerInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.kKickerCurrentLimitAmps);
        kickerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kKickerP)
            .i(ShooterConstants.kKickerI)
            .d(ShooterConstants.kKickerD)
            .feedForward
            .kV(ShooterConstants.kKickerFF * 12.0);
        m_kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_kickerController = m_kickerMotor.getClosedLoopController();

        // --- Pre-Shooter Motor (NEO Vortex + SparkFlex) ---
        SparkFlexConfig preShooterConfig = new SparkFlexConfig();
        preShooterConfig
            .inverted(ShooterConstants.kPreShooterInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kPreShooterCurrentLimitAmps);
        preShooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kPreShooterP)
            .i(ShooterConstants.kPreShooterI)
            .d(ShooterConstants.kPreShooterD)
            .feedForward
            .kV(ShooterConstants.kPreShooterFF * 12.0);
        m_preShooterMotor.configure(preShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_preShooterController = m_preShooterMotor.getClosedLoopController();

        // --- Shooter Primary Motor (NEO Vortex + SparkFlex, CAN 22) ---
        SparkFlexConfig shooterPrimaryConfig = new SparkFlexConfig();
        shooterPrimaryConfig
            .inverted(ShooterConstants.kShooterPrimaryInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kShooterCurrentLimitAmps);
        shooterPrimaryConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kShooterP)
            .i(ShooterConstants.kShooterI)
            .d(ShooterConstants.kShooterD)
            .feedForward
            .kV(ShooterConstants.kShooterFF * 12.0);
        m_shooterPrimaryMotor.configure(shooterPrimaryConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_shooterPrimaryController = m_shooterPrimaryMotor.getClosedLoopController();

        // --- Shooter Secondary Motor (NEO Vortex + SparkFlex, CAN 23) ---
        SparkFlexConfig shooterSecondaryConfig = new SparkFlexConfig();
        shooterSecondaryConfig
            .inverted(ShooterConstants.kShooterSecondaryInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kShooterCurrentLimitAmps);
        shooterSecondaryConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kShooterP)
            .i(ShooterConstants.kShooterI)
            .d(ShooterConstants.kShooterD)
            .feedForward
            .kV(ShooterConstants.kShooterFF * 12.0);
        m_shooterSecondaryMotor.configure(shooterSecondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_shooterSecondaryController = m_shooterSecondaryMotor.getClosedLoopController();
    }

    // -------------------------------------------------------------------------
    // Agitator
    // -------------------------------------------------------------------------

    /** Runs the agitator motor at the configured forward RPM using closed-loop velocity control. */
    public void runAgitator() {
        m_agitatorController.setSetpoint(ShooterConstants.kAgitatorForwardRPM, ControlType.kVelocity);
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
        m_kickerController.setSetpoint(ShooterConstants.kKickerForwardRPM, ControlType.kVelocity);
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
        m_preShooterController.setSetpoint(ShooterConstants.kPreShooterForwardRPM, ControlType.kVelocity);
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
        m_shooterPrimaryController.setSetpoint(ShooterConstants.kShooterForwardRPM, ControlType.kVelocity);
        m_shooterSecondaryController.setSetpoint(ShooterConstants.kShooterForwardRPM, ControlType.kVelocity);
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
        m_agitatorController.setSetpoint(-ShooterConstants.kAgitatorReverseRPM, ControlType.kVelocity);
        m_kickerController.setSetpoint(-ShooterConstants.kKickerReverseRPM, ControlType.kVelocity);
        m_preShooterController.setSetpoint(-ShooterConstants.kPreShooterReverseRPM, ControlType.kVelocity);
        m_shooterPrimaryController.setSetpoint(-ShooterConstants.kShooterReverseRPM, ControlType.kVelocity);
        m_shooterSecondaryController.setSetpoint(-ShooterConstants.kShooterReverseRPM, ControlType.kVelocity);
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
