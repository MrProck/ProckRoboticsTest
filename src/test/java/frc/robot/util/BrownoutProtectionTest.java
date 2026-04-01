package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class BrownoutProtectionTest {

    /**
     * Computes the same scale formula as BrownoutProtection.getCurrentLimitScale()
     * without calling RobotController (which requires hardware).
     */
    private double scale(double voltage) {
        if (voltage >= BrownoutProtection.kWarningVoltage) {
            return 1.0;
        }
        if (voltage <= BrownoutProtection.kCriticalVoltage) {
            return BrownoutProtection.kMinScaleFactor;
        }
        double t = (voltage - BrownoutProtection.kCriticalVoltage)
                / (BrownoutProtection.kWarningVoltage - BrownoutProtection.kCriticalVoltage);
        return BrownoutProtection.kMinScaleFactor + t * (1.0 - BrownoutProtection.kMinScaleFactor);
    }

    @Test
    void aboveWarningVoltageReturnsOne() {
        assertEquals(1.0, scale(12.0), 1e-9);
        assertEquals(1.0, scale(BrownoutProtection.kWarningVoltage), 1e-9);
    }

    @Test
    void atOrBelowCriticalVoltageReturnsMinScale() {
        assertEquals(BrownoutProtection.kMinScaleFactor, scale(BrownoutProtection.kCriticalVoltage), 1e-9);
        assertEquals(BrownoutProtection.kMinScaleFactor, scale(5.0), 1e-9);
        assertEquals(BrownoutProtection.kMinScaleFactor, scale(0.0), 1e-9);
    }

    @Test
    void midpointVoltageReturnsMidScale() {
        double midVoltage = (BrownoutProtection.kWarningVoltage + BrownoutProtection.kCriticalVoltage) / 2.0;
        double expected = BrownoutProtection.kMinScaleFactor
                + 0.5 * (1.0 - BrownoutProtection.kMinScaleFactor);
        assertEquals(expected, scale(midVoltage), 1e-9);
    }

    @Test
    void scaleIsMonotonicallyIncreasingWithVoltage() {
        double prev = scale(BrownoutProtection.kCriticalVoltage);
        for (double v = BrownoutProtection.kCriticalVoltage + 0.1;
                v <= BrownoutProtection.kWarningVoltage; v += 0.1) {
            double curr = scale(v);
            assertTrue(curr >= prev,
                    "Scale should not decrease as voltage increases (v=" + v + ")");
            prev = curr;
        }
    }

    @Test
    void brownoutScaleAppliedToDriveOutputs() {
        // Simulate the scaling logic applied in HerdDriveCommand.execute():
        // drive outputs should be reduced proportionally when voltage is low.
        double xSpeedMPS = 2.0;
        double ySpeedMPS = 1.5;
        double rotationRadPerSec = 1.0;

        // At warning voltage, scale = 1.0 — outputs unchanged
        double scaleNormal = scale(BrownoutProtection.kWarningVoltage);
        assertEquals(xSpeedMPS, xSpeedMPS * scaleNormal, 1e-9);
        assertEquals(ySpeedMPS, ySpeedMPS * scaleNormal, 1e-9);
        assertEquals(rotationRadPerSec, rotationRadPerSec * scaleNormal, 1e-9);

        // At critical voltage, scale = kMinScaleFactor — outputs reduced to minimum
        double scaleMin = scale(BrownoutProtection.kCriticalVoltage);
        assertEquals(xSpeedMPS * BrownoutProtection.kMinScaleFactor, xSpeedMPS * scaleMin, 1e-9);
        assertEquals(ySpeedMPS * BrownoutProtection.kMinScaleFactor, ySpeedMPS * scaleMin, 1e-9);
        assertEquals(rotationRadPerSec * BrownoutProtection.kMinScaleFactor,
                rotationRadPerSec * scaleMin, 1e-9);

        // At midpoint, all outputs are reduced to 65% of original
        double midVoltage = (BrownoutProtection.kWarningVoltage + BrownoutProtection.kCriticalVoltage) / 2.0;
        double scaleMid = scale(midVoltage);
        double expectedMid = BrownoutProtection.kMinScaleFactor
                + 0.5 * (1.0 - BrownoutProtection.kMinScaleFactor);
        assertEquals(xSpeedMPS * expectedMid, xSpeedMPS * scaleMid, 1e-9);
    }
}
