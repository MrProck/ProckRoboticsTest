package frc.robot.subsystems;

import com.andymark.jni.AM_CAN_Color_Sensor;
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
 *  - 3x AndyMark am-5636 CAN color sensors (CAN 16, 17, 18) on RIO CAN bus
 *
 * Safety rule: if ANY sensor detects red or blue, intake is locked out.
 */
public class IntakeSubsystem extends SubsystemBase {

    // Motors
    private final SparkMax  m_extensionMotor = new SparkMax(IntakeConstants.kExtensionMotorID, MotorType.kBrushless);
    private final SparkFlex m_rollerMotor    = new SparkFlex(IntakeConstants.kRollerMotorID,   MotorType.kBrushless);

    // Closed-loop controller for extension position
    private final SparkClosedLoopController m_extensionController;

    // Color sensors (on RIO CAN bus)
    private final AM_CAN_Color_Sensor m_sensorEntry  = new AM_CAN_Color_Sensor(IntakeConstants.kEntrySensorID);
    private final AM_CAN_Color_Sensor m_sensorMiddle = new AM_CAN_Color_Sensor(IntakeConstants.kMiddleSensorID);
    private final AM_CAN_Color_Sensor m_sensorExit   = new AM_CAN_Color_Sensor(IntakeConstants.kExitSensorID);

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
            .forwardSoftLimit(IntakeConstants.kExtensionExtendedPosition)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(IntakeConstants.kExtensionRetractedPosition)
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
     * When true, intake is locked out.
     */
    public boolean isWrongColorDetected() {
        return isRedOrBlue(m_sensorEntry.classifyColor())
            || isRedOrBlue(m_sensorMiddle.classifyColor())
            || isRedOrBlue(m_sensorExit.classifyColor());
    }

    private boolean isRedOrBlue(String color) {
        return "Red".equalsIgnoreCase(color) || "Blue".equalsIgnoreCase(color);
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
        SmartDashboard.putString("Sensor Entry",  m_sensorEntry.classifyColor());
        SmartDashboard.putString("Sensor Middle", m_sensorMiddle.classifyColor());
        SmartDashboard.putString("Sensor Exit",   m_sensorExit.classifyColor());
        SmartDashboard.putBoolean("Intake Locked",  m_intakeLocked);
        SmartDashboard.putNumber("Extension Pos",   getExtensionPosition());
    }
}
