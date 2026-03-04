package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveModuleStateOptimizeTest {

    @Test
    void optimizeFlipsSpeedAndAngleWhenDeltaOver90Degrees() {
        // Desired angle is 180 degrees away from current — optimize should flip
        SwerveModuleState desired = new SwerveModuleState(1.0, Rotation2d.fromDegrees(180.0));
        Rotation2d currentAngle = Rotation2d.fromDegrees(0.0);

        SwerveModuleState optimized = SwerveModuleState.optimize(desired, currentAngle);

        // Speed should be negated and angle should be adjusted
        assertEquals(-1.0, optimized.speedMetersPerSecond, 1e-9);
        assertEquals(0.0, optimized.angle.getDegrees(), 1e-9);
    }

    @Test
    void optimizeNoChangeWhenWithin90Degrees() {
        // Desired angle is within 90 degrees — no change expected
        SwerveModuleState desired = new SwerveModuleState(1.0, Rotation2d.fromDegrees(45.0));
        Rotation2d currentAngle = Rotation2d.fromDegrees(0.0);

        SwerveModuleState optimized = SwerveModuleState.optimize(desired, currentAngle);

        assertEquals(1.0, optimized.speedMetersPerSecond, 1e-9);
        assertEquals(45.0, optimized.angle.getDegrees(), 1e-9);
    }

    @Test
    void optimizeZeroSpeedStateIsHandled() {
        // Zero speed state — angle flip doesn't matter but should not throw
        SwerveModuleState desired = new SwerveModuleState(0.0, Rotation2d.fromDegrees(180.0));
        Rotation2d currentAngle = Rotation2d.fromDegrees(0.0);

        SwerveModuleState optimized = SwerveModuleState.optimize(desired, currentAngle);

        // Negating 0 is still 0
        assertEquals(0.0, optimized.speedMetersPerSecond, 1e-9);
    }
}
