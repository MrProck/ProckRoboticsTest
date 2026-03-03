package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;

/**
 * Shared utility methods for REV Robotics motor controllers.
 */
public final class REVUtil {

    private REVUtil() {}

    /**
     * Checks a REVLibError and logs a warning if it is not OK.
     *
     * @param label A human-readable label for the operation
     * @param error The REVLibError returned by the REV API call
     */
    public static void checkREV(String label, REVLibError error) {
        if (error != REVLibError.kOk) {
            System.err.println("[REVUtil] " + label + " failed: " + error);
        }
    }

    /**
     * Waits 200 ms (REV-recommended delay) then burns flash, logging any error.
     *
     * @param motor The motor whose flash to burn
     * @param label A human-readable label for logging
     */
    public static void burnFlashWithDelay(CANSparkBase motor, String label) {
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        checkREV(label, motor.burnFlash());
    }
}
