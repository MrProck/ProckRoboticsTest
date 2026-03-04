package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.SwerveConstants;

/**
 * Tests the swerve module conversion formulas using the robot's configured constants.
 *
 * <p>The SwerveModule configures the TalonFX's {@code SensorToMechanismRatio} to
 * {@code kDriveGearRatio}, so the encoder reports directly in <em>wheel</em> rotations.
 * All conversions are therefore:
 *
 * <ul>
 *   <li>Distance: {@code wheelRotations × kWheelCircumferenceMeters}</li>
 *   <li>Speed:    {@code wheelRPS × kWheelCircumferenceMeters}</li>
 * </ul>
 *
 * When computing values from raw <em>motor</em> rotations, the gear ratio must
 * be divided out first:
 *
 * <ul>
 *   <li>1 motor rotation → {@code kWheelDiameterMeters × π / kDriveGearRatio} meters</li>
 *   <li>1 motor RPM     → {@code kWheelDiameterMeters × π / kDriveGearRatio / 60} m/s</li>
 *   <li>1 steer motor rotation → {@code 2π / kSteerGearRatio} radians of wheel turn</li>
 * </ul>
 */
class SwerveModuleMathTest {

    private static final double TOLERANCE = 1e-9;

    /**
     * One raw drive motor rotation corresponds to
     * {@code kWheelDiameterMeters * PI / kDriveGearRatio} meters of travel.
     */
    @Test
    void driveRotationsToMeters() {
        double expectedMetersPerMotorRotation =
            SwerveConstants.kWheelDiameterMeters * Math.PI / SwerveConstants.kDriveGearRatio;

        // One wheel rotation = kWheelCircumferenceMeters; one motor rotation = 1/kDriveGearRatio wheel rotations
        double actualMetersPerMotorRotation =
            SwerveConstants.kWheelCircumferenceMeters / SwerveConstants.kDriveGearRatio;

        assertEquals(expectedMetersPerMotorRotation, actualMetersPerMotorRotation, TOLERANCE,
            "1 motor rotation should equal kWheelDiameterMeters * PI / kDriveGearRatio meters");
    }

    /**
     * One drive motor RPM corresponds to
     * {@code kWheelDiameterMeters * PI / kDriveGearRatio / 60} meters per second.
     */
    @Test
    void driveRPMToMetersPerSecond() {
        double motorRPM = 1.0;
        double expectedMPS =
            motorRPM * SwerveConstants.kWheelDiameterMeters * Math.PI / SwerveConstants.kDriveGearRatio / 60.0;

        double actualMPS = motorRPM * SwerveConstants.kWheelCircumferenceMeters / SwerveConstants.kDriveGearRatio / 60.0;

        assertEquals(expectedMPS, actualMPS, TOLERANCE,
            "1 motor RPM should equal kWheelCircumferenceMeters / kDriveGearRatio / 60 m/s");
    }

    /**
     * One raw steer motor rotation corresponds to
     * {@code 2 * PI / kSteerGearRatio} radians of wheel azimuth change.
     */
    @Test
    void turningRotationsToRadians() {
        double motorRotations = 1.0;
        double expectedRadians = 2.0 * Math.PI * motorRotations / SwerveConstants.kSteerGearRatio;

        // The steer gear ratio converts motor rotations to wheel azimuth rotations;
        // multiply by 2π to get radians.
        double actualRadians = motorRotations / SwerveConstants.kSteerGearRatio * 2.0 * Math.PI;

        assertEquals(expectedRadians, actualRadians, TOLERANCE,
            "1 steer motor rotation should equal 2*PI / kSteerGearRatio radians");
    }

    /**
     * Sanity-checks that the wheel circumference constant is consistent with the diameter.
     */
    @Test
    void wheelCircumferenceMatchesDiameterTimesPi() {
        assertEquals(
            SwerveConstants.kWheelDiameterMeters * Math.PI,
            SwerveConstants.kWheelCircumferenceMeters,
            TOLERANCE,
            "kWheelCircumferenceMeters must equal kWheelDiameterMeters * PI");
    }
}
