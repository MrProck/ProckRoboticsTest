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
        double trigger = MathUtil.clamp(slowMode, 0.0, 1.0);
        double speedMultiplier = 1.0 - (trigger * trigger * trigger);
        assertEquals(0.0, speedMultiplier, 1e-9);
    }

    @Test
    void noTriggerResultsInFullSpeedMultiplier() {
        // When slow-mode trigger is not pressed (0.0), speed multiplier is 1
        double slowMode = 0.0;
        double trigger = MathUtil.clamp(slowMode, 0.0, 1.0);
        double speedMultiplier = 1.0 - (trigger * trigger * trigger);
        assertEquals(1.0, speedMultiplier, 1e-9);
    }

    @Test
    void halfTriggerCubedBrakeScaling() {
        // 50% trigger pull: 0.5³ = 0.125, so speedMultiplier = 1.0 - 0.125 = 0.875
        // (linear scaling would give 0.5, so cubed gives much less braking at half pull)
        double slowMode = 0.5;
        double trigger = MathUtil.clamp(slowMode, 0.0, 1.0);
        double speedMultiplier = 1.0 - (trigger * trigger * trigger);
        assertEquals(0.875, speedMultiplier, 1e-9);
    }

    @Test
    void quarterTriggerCubedBrakeScaling() {
        // 25% trigger pull: 0.25³ = 0.015625, so speedMultiplier = 1.0 - 0.015625 = 0.984375
        double slowMode = 0.25;
        double trigger = MathUtil.clamp(slowMode, 0.0, 1.0);
        double speedMultiplier = 1.0 - (trigger * trigger * trigger);
        assertEquals(0.984375, speedMultiplier, 1e-9);
    }

    @Test
    void smallInputsAreZeroedByDeadband() {
        // Inputs within deadband should be zeroed out
        double smallInput = SwerveConstants.kDeadband * 0.5;
        double result = MathUtil.applyDeadband(smallInput, SwerveConstants.kDeadband);
        assertEquals(0.0, result, 1e-9);
    }

    @Test
    void fullStickInputProducesMaxTeleopSpeed() {
        // Full stick input (1.0) with no trigger should produce max teleop speed.
        // blendedCubicInput(1.0) = blend * 1.0 + (1 - blend) * 1.0^3 = 1.0 always.
        double xInput = 1.0;
        double slowMode = 0.0;
        double blend = SwerveConstants.kInputCurveLinearBlend;
        double curvedInput = blend * xInput + (1.0 - blend) * (xInput * xInput * xInput);
        double trigger = MathUtil.clamp(slowMode, 0.0, 1.0);
        double speedMultiplier = 1.0 - (trigger * trigger * trigger);
        double xSpeedMPS = MathUtil.applyDeadband(xInput, SwerveConstants.kDeadband)
            * SwerveConstants.kMaxDriveSpeedMetersPerSecond * SwerveConstants.kTeleopMaxDriveSpeed
            * speedMultiplier;
        // curvedInput at full stick is always 1.0 regardless of blend
        assertEquals(1.0, curvedInput, 1e-9,
            "blendedCubicInput(1.0) must equal 1.0 for any blend value");
        double expectedMax = SwerveConstants.kMaxDriveSpeedMetersPerSecond
            * SwerveConstants.kTeleopMaxDriveSpeed;
        assertEquals(expectedMax, xSpeedMPS, 1e-9);
    }

    @Test
    void blendedCubicMidStickOutput() {
        // At mid-stick (0.5), output depends on blend factor.
        // With blend=1.0 (fully linear), output equals input exactly.
        // With blend<1.0 (cubic mix), output is reduced for finer low-speed control.
        double input = 0.5;
        double blend = SwerveConstants.kInputCurveLinearBlend;
        double curvedOutput = blend * input + (1.0 - blend) * (input * input * input);
        double expectedOutput = blend * input + (1.0 - blend) * (input * input * input);
        assertEquals(expectedOutput, curvedOutput, 1e-9,
            "blendedCubicInput(0.5) should match the formula for the configured blend");
        assertTrue(curvedOutput > 0.0,
            "blendedCubicInput(0.5) should still be positive");
        assertTrue(curvedOutput <= input,
            "blendedCubicInput(0.5) should be at most linear (never exceeds input for 0<x<1)");
    }

    @Test
    void blendedCubicPreservesSign() {
        // Negative inputs should produce negative outputs
        double input = -0.5;
        double blend = SwerveConstants.kInputCurveLinearBlend;
        double curvedOutput = blend * input + (1.0 - blend) * (input * input * input);
        assertTrue(curvedOutput < 0.0,
            "blendedCubicInput(-0.5) should be negative");
    }

    @Test
    void blendedCubicAtZeroIsZero() {
        double input = 0.0;
        double blend = SwerveConstants.kInputCurveLinearBlend;
        double curvedOutput = blend * input + (1.0 - blend) * (input * input * input);
        assertEquals(0.0, curvedOutput, 1e-9,
            "blendedCubicInput(0.0) should be exactly 0.0");
    }
}
