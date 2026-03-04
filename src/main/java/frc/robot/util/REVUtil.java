package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Shared utility methods for REV motor controllers (CANSparkMax / CANSparkFlex).
 *
 * <p>Provides a single point for error checking and common motor configuration
 * so that subsystems do not repeat boilerplate setup code.
 */
public final class REVUtil {

    private REVUtil() {}

    /**
     * Checks a {@link REVLibError} return value and reports a warning to the
     * Driver Station if the error is not {@code kOk}.
     *
     * @param error   The error returned by a REV API call.
     * @param context A human-readable description of the operation (used in the warning message).
     * @return The original error, allowing calls to be chained.
     */
    public static REVLibError check(REVLibError error, String context) {
        if (error != REVLibError.kOk) {
            DriverStation.reportWarning(
                "[REVUtil] " + context + " failed: " + error.toString(), false);
        }
        return error;
    }

    /**
     * Applies common configuration to a {@link SparkMax} motor controller.
     *
     * <p>Restores factory defaults, sets the inversion, applies a smart current
     * limit, and burns the configuration to flash.
     *
     * @param motor        The SparkMax to configure.
     * @param inverted     Whether the motor output should be inverted.
     * @param currentLimit Smart current limit in amps.
     */
    public static void configureSparkMax(SparkMax motor, boolean inverted, int currentLimit) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(currentLimit);
        check(
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
            "SparkMax (ID " + motor.getDeviceId() + ") configure");
    }

    /**
     * Applies common configuration to a {@link SparkFlex} motor controller.
     *
     * <p>Restores factory defaults, sets the inversion, applies a smart current
     * limit, and burns the configuration to flash.
     *
     * @param motor        The SparkFlex to configure.
     * @param inverted     Whether the motor output should be inverted.
     * @param currentLimit Smart current limit in amps.
     */
    public static void configureSparkFlex(SparkFlex motor, boolean inverted, int currentLimit) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(inverted)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(currentLimit);
        check(
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
            "SparkFlex (ID " + motor.getDeviceId() + ") configure");
    }
}
