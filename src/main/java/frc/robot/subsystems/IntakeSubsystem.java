package frc.robot.subsystems;

import com.andymark.jni.AM_CAN_Color_Sensor;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
    private final CANSparkMax  m_extensionMotor = new CANSparkMax(IntakeConstants.kExtensionMotorID, MotorType.kBrushless);
    private final CANSparkFlex m_rollerMotor    = new CANSparkFlex(IntakeConstants.kRollerMotorID,   MotorType.kBrushless);

    // PID controller for extension position
    private final SparkPIDController m_extensionPID;

    // Color sensors (on RIO CAN bus)
    private final AM_CAN_Color_Sensor m_sensorEntry  = new AM_CAN_Color_Sensor(IntakeConstants.kEntrySensorID);
    private final AM_CAN_Color_Sensor m_sensorMiddle = new AM_CAN_Color_Sensor(IntakeConstants.kMiddleSensorID);
    private final AM_CAN_Color_Sensor m_sensorExit   = new AM_CAN_Color_Sensor(IntakeConstants.kExitSensorID);

    // Lockout state — true if a wrong-color game piece was detected
    private boolean m_intakeLocked = false;

    public IntakeSubsystem() {
        // --- Extension Motor (SparkMax) ---
        checkREV("Extension restoreFactoryDefaults", m_extensionMotor.restoreFactoryDefaults());
        checkREV("Extension setIdleMode", m_extensionMotor.setIdleMode(IdleMode.kBrake));
        checkREV("Extension setSmartCurrentLimit", m_extensionMotor.setSmartCurrentLimit(IntakeConstants.kExtensionCurrentLimitAmps));
        m_extensionMotor.setInverted(IntakeConstants.kExtensionMotorInverted);

        m_extensionPID = m_extensionMotor.getPIDController();
        checkREV("Extension PID P", m_extensionPID.setP(IntakeConstants.kExtensionP));
        checkREV("Extension PID I", m_extensionPID.setI(IntakeConstants.kExtensionI));
        checkREV("Extension PID D", m_extensionPID.setD(IntakeConstants.kExtensionD));
        checkREV("Extension PID FF", m_extensionPID.setFF(IntakeConstants.kExtensionFF));

        checkREV("Extension soft limit forward set", m_extensionMotor.setSoftLimit(SoftLimitDirection.kForward, (float) IntakeConstants.kExtensionExtendedPosition));
        checkREV("Extension soft limit reverse set", m_extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) IntakeConstants.kExtensionRetractedPosition));
        checkREV("Extension soft limit forward", m_extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true));
        checkREV("Extension soft limit reverse", m_extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true));

        checkREV("Extension setPosition", m_extensionMotor.getEncoder().setPosition(0.0));
        checkREV("Extension setPositionConversionFactor", m_extensionMotor.getEncoder().setPositionConversionFactor(IntakeConstants.kExtensionPositionConversionFactor));
        burnFlashWithDelay(m_extensionMotor, "Extension burnFlash");

        // --- Roller Motor (SparkFlex) ---
        checkREV("Roller restoreFactoryDefaults", m_rollerMotor.restoreFactoryDefaults());
        checkREV("Roller setIdleMode", m_rollerMotor.setIdleMode(IdleMode.kCoast));
        checkREV("Roller setSmartCurrentLimit", m_rollerMotor.setSmartCurrentLimit(IntakeConstants.kRollerCurrentLimitAmps));
        m_rollerMotor.setInverted(IntakeConstants.kRollerMotorInverted);
        burnFlashWithDelay(m_rollerMotor, "Roller burnFlash");
    }

    /**
     * Checks a REVLibError and logs a warning if it is not OK.
     *
     * @param label   A human-readable label for the operation (e.g., "Extension restoreFactoryDefaults")
     * @param error   The REVLibError returned by the REV API call
     */
    private static void checkREV(String label, REVLibError error) {
        if (error != REVLibError.kOk) {
            System.err.println("[IntakeSubsystem] " + label + " failed: " + error);
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
        m_extensionPID.setReference(
            IntakeConstants.kExtensionExtendedPosition, ControlType.kPosition);
    }

    /** Retracts the intake arm to the stowed position. */
    public void retract() {
        m_extensionPID.setReference(
            IntakeConstants.kExtensionRetractedPosition, ControlType.kPosition);
    }

    /** Holds the extension arm at its current position. */
    public void holdPosition() {
        m_extensionPID.setReference(
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
