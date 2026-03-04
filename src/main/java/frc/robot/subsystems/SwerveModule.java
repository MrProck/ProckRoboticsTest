package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.SwerveConstants;

/**
 * Represents a single SDS MK4 L2 swerve module.
 * Drive motor: Falcon 500 (TalonFX)
 * Steer motor: Falcon 500 (TalonFX)
 * Absolute encoder: CANcoder
 * All devices on the CANivore bus.
 */
public class SwerveModule {

    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;
    private final CANcoder m_CANcoder;

    private final VelocityVoltage m_driveVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage m_steerPositionControl = new PositionVoltage(0).withSlot(0);

    private final double m_CANcoderOffset;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorID   CAN ID of the drive TalonFX
     * @param steerMotorID   CAN ID of the steer TalonFX
     * @param CANcoderID     CAN ID of the CANcoder
     * @param CANcoderOffset Absolute offset of the CANcoder (rotations) — tune with Tuner X
     * @param driveInverted  Whether the drive motor is inverted
     * @param steerInverted  Whether the steer motor is inverted
     */
    public SwerveModule(
        int driveMotorID,
        int steerMotorID,
        int CANcoderID,
        double CANcoderOffset,
        boolean driveInverted,
        boolean steerInverted
    ) {
        m_CANcoderOffset = CANcoderOffset;

        m_driveMotor = new TalonFX(driveMotorID, SwerveConstants.kCANivoreBus);
        m_steerMotor = new TalonFX(steerMotorID, SwerveConstants.kCANivoreBus);
        m_CANcoder   = new CANcoder(CANcoderID,  SwerveConstants.kCANivoreBus);

        configureCANcoder();
        configureDriveMotor(driveInverted);
        configureSteerMotor(steerInverted);
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    private void configureCANcoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset    = m_CANcoderOffset;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        applyWithRetry(() -> m_CANcoder.getConfigurator().apply(config), 5);
    }

    private void configureDriveMotor(boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current limit
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = SwerveConstants.kDriveCurrentLimitAmps;

        // Inversion & neutral mode
        config.MotorOutput.Inverted    = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Velocity PID (slot 0)
        config.Slot0.kP = SwerveConstants.kDriveP;
        config.Slot0.kI = SwerveConstants.kDriveI;
        config.Slot0.kD = SwerveConstants.kDriveD;
        config.Slot0.kV = SwerveConstants.kDriveFF;

        // Gear ratio so encoder reports in wheel rotations
        config.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;

        applyWithRetry(() -> m_driveMotor.getConfigurator().apply(config), 5);
    }

    private void configureSteerMotor(boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current limit — 40A
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = SwerveConstants.kSteerCurrentLimitAmps;

        // Inversion & neutral mode
        config.MotorOutput.Inverted    = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Position PID (slot 0)
        config.Slot0.kP = SwerveConstants.kSteerP;
        config.Slot0.kI = SwerveConstants.kSteerI;
        config.Slot0.kD = SwerveConstants.kSteerD;
        config.Slot0.kV = SwerveConstants.kSteerFF;

        // Fuse CANcoder as remote feedback sensor
        config.Feedback.FeedbackSensorSource   = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = m_CANcoder.getDeviceID();
        config.Feedback.SensorToMechanismRatio  = 1.0;
        config.Feedback.RotorToSensorRatio      = SwerveConstants.kSteerGearRatio;

        // Continuous wrap — always take shortest path to target angle
        config.ClosedLoopGeneral.ContinuousWrap = true;

        applyWithRetry(() -> m_steerMotor.getConfigurator().apply(config), 5);
    }

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    /**
     * Returns the current state of the module (velocity m/s, angle).
     * <p>
     * Note: The drive motor's {@code SensorToMechanismRatio} is set to {@code kDriveGearRatio},
     * so {@code getVelocity()} already reports in <b>wheel rotations/sec</b> (post-gear-reduction).
     * We only need to multiply by wheel circumference to get m/s — no additional gear-ratio
     * division is needed here.
     */
    public SwerveModuleState getState() {
        double velocityMPS = m_driveMotor.getVelocity().getValueAsDouble()
            * SwerveConstants.kWheelCircumferenceMeters;
        Rotation2d angle = Rotation2d.fromRotations(
            m_CANcoder.getAbsolutePosition().getValueAsDouble());
        return new SwerveModuleState(velocityMPS, angle);
    }

    /**
     * Returns the current position of the module (distance m, angle).
     * <p>
     * Like {@link #getState()}, the drive encoder already reports in wheel rotations
     * thanks to {@code SensorToMechanismRatio}, so we multiply by circumference only.
     */
    public SwerveModulePosition getPosition() {
        double distanceMeters = m_driveMotor.getPosition().getValueAsDouble()
            * SwerveConstants.kWheelCircumferenceMeters;
        Rotation2d angle = Rotation2d.fromRotations(
            m_CANcoder.getAbsolutePosition().getValueAsDouble());
        return new SwerveModulePosition(distanceMeters, angle);
    }

    /**
     * Commands the module to a desired state (optimized).
     *
     * @param desiredState The desired SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Suppress small movements to prevent wheel jittering when stationary
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            m_driveMotor.stopMotor();
            return;
        }

        SwerveModuleState optimized = SwerveModuleState.optimize(
            desiredState,
            Rotation2d.fromRotations(m_CANcoder.getAbsolutePosition().getValueAsDouble())
        );

        // Drive velocity in wheel rotations/s (SensorToMechanismRatio handles gear reduction)
        double driveRotationsPerSecond =
            optimized.speedMetersPerSecond / SwerveConstants.kWheelCircumferenceMeters;
        m_driveMotor.setControl(m_driveVelocityControl.withVelocity(driveRotationsPerSecond));

        // Steer position in rotations
        m_steerMotor.setControl(m_steerPositionControl.withPosition(
            optimized.angle.getRotations()));
    }

    /** Stops both drive and steer motors. */
    public void stop() {
        m_driveMotor.stopMotor();
        m_steerMotor.stopMotor();
    }

    /**
     * Applies a device configuration with retry logic.
     * Retries up to {@code maxRetries} times if the apply does not return OK.
     */
    private static StatusCode applyWithRetry(
            java.util.function.Supplier<StatusCode> applyCall, int maxRetries) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < maxRetries; i++) {
            status = applyCall.get();
            if (status == StatusCode.OK) {
                return status;
            }
            System.err.println("[SwerveModule] Config apply failed (attempt " + (i + 1) + "): " + status);
        }
        System.err.println("[SwerveModule] Config apply failed after " + maxRetries + " attempts: " + status);
        return status;
    }
}
