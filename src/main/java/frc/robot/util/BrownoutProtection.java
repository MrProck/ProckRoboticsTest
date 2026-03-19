package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Monitors battery voltage and provides a scaling factor for current limits
 * to prevent brownouts during high-draw situations.
 *
 * <p>When voltage drops below kWarningVoltage, the scale factor linearly
 * decreases from 1.0 to kMinScaleFactor at kCriticalVoltage.
 */
public final class BrownoutProtection {

    private BrownoutProtection() {}

    /** Voltage at which current limiting begins reducing (volts). */
    public static final double kWarningVoltage = 8.5;
    /** Voltage at which current limits are at their minimum scale (volts). */
    public static final double kCriticalVoltage = 7.0;
    /** Minimum current limit scale factor (0.3 = 30% of normal). */
    public static final double kMinScaleFactor = 0.3;

    /**
     * Returns a scale factor (0.3–1.0) for current limits based on battery voltage.
     * Above kWarningVoltage returns 1.0. Below kCriticalVoltage returns kMinScaleFactor.
     * Linear interpolation between the two thresholds.
     */
    public static double getCurrentLimitScale() {
        double voltage = RobotController.getBatteryVoltage();
        if (voltage >= kWarningVoltage) {
            return 1.0;
        }
        if (voltage <= kCriticalVoltage) {
            return kMinScaleFactor;
        }
        // Linear interpolation
        double t = (voltage - kCriticalVoltage) / (kWarningVoltage - kCriticalVoltage);
        return kMinScaleFactor + t * (1.0 - kMinScaleFactor);
    }

    /**
     * Returns true if the battery voltage is below the warning threshold.
     */
    public static boolean isBrownoutWarning() {
        return RobotController.getBatteryVoltage() < kWarningVoltage;
    }

    /**
     * Publishes battery voltage and brownout status to SmartDashboard.
     * Call this from Robot.robotPeriodic().
     */
    public static void publishTelemetry() {
        double voltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("Battery/Voltage", voltage);
        SmartDashboard.putBoolean("Battery/BrownoutWarning", voltage < kWarningVoltage);
        SmartDashboard.putNumber("Battery/CurrentLimitScale", getCurrentLimitScale());
    }
}
