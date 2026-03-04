package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;

class ConstantsValidationTest {

    /**
     * Collects every {@code static final int} field from all {@code *Constants} inner classes
     * whose name starts with {@code k} and contains {@code Id}, {@code ID}, or {@code Port}.
     * Validates that no two fields on the same CAN bus have the same value.
     */
    @Test
    void noDuplicateCanIdsAcrossConstants() throws Exception {
        // Separate the CANivore-bus IDs from RIO-bus IDs because they use different buses
        List<Integer> canivoreIds = collectIds(Constants.SwerveConstants.class);
        List<Integer> rioIds = new ArrayList<>();
        rioIds.addAll(collectIds(Constants.IntakeConstants.class));
        rioIds.addAll(collectIds(Constants.ShooterConstants.class));

        // Check each bus independently for duplicates
        assertNoDuplicates(canivoreIds, "CANivore bus");
        assertNoDuplicates(rioIds, "RIO CAN bus");
    }

    @Test
    void swerveSpeedConstantsArePositive() {
        assertTrue(SwerveConstants.kMaxDriveSpeedMetersPerSecond > 0.0,
            "kMaxDriveSpeedMetersPerSecond must be positive");
        assertTrue(SwerveConstants.kMaxAngularVelocityRadiansPerSecond > 0.0,
            "kMaxAngularVelocityRadiansPerSecond must be positive");
        assertTrue(SwerveConstants.kTeleopMaxDriveSpeed > 0.0,
            "kTeleopMaxDriveSpeed must be positive");
        assertTrue(SwerveConstants.kTeleopMaxAngularSpeed > 0.0,
            "kTeleopMaxAngularSpeed must be positive");
    }

    @Test
    void deadbandIsInValidRange() {
        assertTrue(SwerveConstants.kDeadband > 0.0 && SwerveConstants.kDeadband < 1.0,
            "kDeadband must be between 0 and 1 (exclusive)");
        assertTrue(OIConstants.kDriveDeadband > 0.0 && OIConstants.kDriveDeadband < 1.0,
            "kDriveDeadband must be between 0 and 1 (exclusive)");
    }

    @Test
    void wheelDiameterIsPositive() {
        assertTrue(SwerveConstants.kWheelDiameterMeters > 0.0,
            "kWheelDiameterMeters must be positive");
    }

    @Test
    void gearRatiosArePositive() {
        assertTrue(SwerveConstants.kDriveGearRatio > 0.0,
            "kDriveGearRatio must be positive");
        assertTrue(SwerveConstants.kSteerGearRatio > 0.0,
            "kSteerGearRatio must be positive");
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private List<Integer> collectIds(Class<?> clazz) throws IllegalAccessException {
        List<Integer> ids = new ArrayList<>();
        for (Field field : clazz.getFields()) {
            if (!Modifier.isStatic(field.getModifiers())) continue;
            if (field.getType() != int.class && field.getType() != Integer.class) continue;
            String name = field.getName();
            if (name.startsWith("k") && (name.contains("Id") || name.contains("ID") || name.contains("Port"))) {
                ids.add((Integer) field.get(null));
            }
        }
        return ids;
    }

    private void assertNoDuplicates(List<Integer> ids, String busName) {
        Map<Integer, Integer> seen = new HashMap<>();
        for (int id : ids) {
            assertFalse(seen.containsKey(id),
                "Duplicate CAN ID " + id + " found on " + busName);
            seen.put(id, id);
        }
    }
}
