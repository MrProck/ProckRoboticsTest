package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.SwerveConstants;

class TeleopDriveCommandTest {

    @Test
    void fullTriggerResultsInZeroSpeedMultiplier() {
        // When slow-mode trigger is fully depressed (1.0), speed multiplier is 0
        double slowMode = 1.0;
        double speedMultiplier = 1.0 - MathUtil.clamp(slowMode, 0.0, 1.0);
        assertEquals(0.0, speedMultiplier, 1e-9);
    }

    @Test
    void noTriggerResultsInFullSpeedMultiplier() {
        // When slow-mode trigger is not pressed (0.0), speed multiplier is 1
        double slowMode = 0.0;
        double speedMultiplier = 1.0 - MathUtil.clamp(slowMode, 0.0, 1.0);
        assertEquals(1.0, speedMultiplier, 1e-9);
    }

    @Test
    void smallInputsAreZeroedByDeadband() {
        // Inputs within deadband should be zeroed out
        double smallInput = SwerveConstants.kDeadband * 0.5;
        double result = MathUtil.applyDeadband(smallInput, SwerveConstants.kDeadband);
        assertEquals(0.0, result, 1e-9);
    }

    @Test
    void fullStickInputProducesProportionalOutput() {
        // Full stick input (1.0) with no trigger should produce max teleop speed
        double xInput = 1.0;
        double slowMode = 0.0;
        double speedMultiplier = 1.0 - MathUtil.clamp(slowMode, 0.0, 1.0);
        double xSpeedMPS = MathUtil.applyDeadband(xInput, SwerveConstants.kDeadband)
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed
            * speedMultiplier;

        double expectedMax = SwerveConstants.kMaxDriveSpeedMetersPerSecond
            * SwerveConstants.kTeleopMaxDriveSpeed;
        assertEquals(expectedMax, xSpeedMPS, 1e-9);
    }
}
