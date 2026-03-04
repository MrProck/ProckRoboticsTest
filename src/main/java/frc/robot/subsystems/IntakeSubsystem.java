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

import frc.robot.Constants.IntakeConstants;

/**
 * Intake subsystem with:
 *  - Extension arm (NEO + SparkMax, CAN 14)
 *  - Intake roller (NEO Vortex + SparkFlex, CAN 15)
 *
 * Color sensor lockout is disabled (AndyMark vendordep not available).
 * isWrongColorDetected() always returns false.
 */
public class IntakeSubsystem extends SubsystemBase {

    // Motors
    private final SparkMax  m_extensionMotor = new SparkMax(IntakeConstants.kExtensionMotorID, MotorType.kBrushless);
    private final SparkFlex m_rollerMotor    = new SparkFlex(IntakeConstants.kRollerMotorID,   MotorType.kBrushless);

    // Closed-loop controller for extension position
    private final SparkClosedLoopController m_extensionController;

    // Lockout state — true if a wrong-color game piece was detected
    private boolean m_intakeLocked = false;

    public IntakeSubsystem() {
        // --- Extension Motor (SparkMax) ---
        SparkMaxConfig extensionConfig = new SparkMaxConfig();
        extensionConfig
            .inverted(IntakeConstants.kExtensionMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.kExtensionCurrentLimitAmps);
        extensionConfig.encoder
            .positionConversionFactor(IntakeConstants.kExtensionPositionConversionFactor);
        extensionConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(IntakeConstants.kExtensionP)
            .i(IntakeConstants.kExtensionI)
            .d(IntakeConstants.kExtensionD);
        extensionConfig.softLimit
            .forwardSoftLimit((float) IntakeConstants.kExtensionExtendedPosition)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit((float) IntakeConstants.kExtensionRetractedPosition)
            .reverseSoftLimitEnabled(true);
        m_extensionMotor.configure(extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_extensionMotor.getEncoder().setPosition(0.0);
        m_extensionController = m_extensionMotor.getClosedLoopController();

        // --- Roller Motor (SparkFlex) ---
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig
            .inverted(IntakeConstants.kRollerMotorInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(IntakeConstants.kRollerCurrentLimitAmps);
        m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // -------------------------------------------------------------------------
    // Color Sensor Logic
    // -------------------------------------------------------------------------

    /**
     * Returns true if any sensor sees red or blue (wrong game piece).
     * Color sensor (AndyMark am-5636) vendordep is not available; always returns false.
     */
    public boolean isWrongColorDetected() {
        return false;
    }

    /** Clears the intake lockout (call after ejecting the wrong piece). */
    public void clearLockout() {
        m_intakeLocked = false;
    }

    /** Returns true if the intake is currently locked out. */
    public boolean isLocked() {
        return m_intakeLocked;
    }

    // -------------------------------------------------------------------------
    // Extension
    // -------------------------------------------------------------------------

    /** Extends the intake arm to the deployed position. */
    public void extend() {
        m_extensionController.setSetpoint(
            IntakeConstants.kExtensionExtendedPosition, ControlType.kPosition);
    }

    /** Retracts the intake arm to the stowed position. */
    public void retract() {
        m_extensionController.setSetpoint(
            IntakeConstants.kExtensionRetractedPosition, ControlType.kPosition);
    }

    /** Holds the extension arm at its current position. */
    public void holdPosition() {
        m_extensionController.setSetpoint(
            m_extensionMotor.getEncoder().getPosition(), ControlType.kPosition);
    }

    /** Returns the current extension position in rotations. */
    public double getExtensionPosition() {
        return m_extensionMotor.getEncoder().getPosition();
    }

    // -------------------------------------------------------------------------
    // Roller
    // -------------------------------------------------------------------------

    /**
     * Runs the intake roller forward (intaking).
     * Does nothing if intake is locked out.
     */
    public void runIntake() {
        if (m_intakeLocked) {
            m_rollerMotor.stopMotor();
            return;
        }
        m_rollerMotor.set(IntakeConstants.kRollerForwardSpeed);
    }

    /**
     * Runs the intake roller in reverse (ejecting).
     * Always allowed — used to eject wrong game piece and clear lockout.
     */
    public void runEject() {
        m_rollerMotor.set(IntakeConstants.kRollerReverseSpeed);
    }

    /** Stops the intake roller. */
    public void stopRoller() {
        m_rollerMotor.stopMotor();
    }

    /**
     * Retracts the intake arm and stops the roller motor.
     * Safe to call at any time, including when disabled.
     */
    public void safeRetract() {
        retract();
        stopRoller();
    }

    /**
     * Returns true if the intake arm is currently in the extended position.
     * Uses a threshold of half the extension range to determine state.
     */
    public boolean isExtended() {
        // Half the full extension range (25 rotations) distinguishes "extended" from "retracted".
        // Any position above the midpoint is considered extended.
        return getExtensionPosition() > IntakeConstants.kExtensionExtendedPosition / 2.0;
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Update lockout — once locked, stays locked until clearLockout() is called
        if (isWrongColorDetected()) {
            m_intakeLocked = true;
            m_rollerMotor.stopMotor();
        }

        // SmartDashboard telemetry
        SmartDashboard.putBoolean("Intake Locked",  m_intakeLocked);
        SmartDashboard.putNumber("Extension Pos",   getExtensionPosition());
    }
}
