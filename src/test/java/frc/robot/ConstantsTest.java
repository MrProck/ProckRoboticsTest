package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.util.HashSet;
import java.util.Set;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;

class ConstantsTest {

    @Test
    void wheelCircumferenceEqualsDiameterTimesPi() {
        assertEquals(
            SwerveConstants.kWheelDiameterMeters * Math.PI,
            SwerveConstants.kWheelCircumferenceMeters,
            1e-9);
    }

    @Test
    void driveGearRatioIsPositive() {
        assertTrue(SwerveConstants.kDriveGearRatio > 0.0);
    }

    @Test
    void steerGearRatioIsPositive() {
        assertTrue(SwerveConstants.kSteerGearRatio > 0.0);
    }

    @Test
    void maxDriveSpeedIsPositive() {
        assertTrue(SwerveConstants.kMaxDriveSpeedMetersPerSecond > 0.0);
    }

    @Test
    void swerveKinematicsIsNotNull() {
        assertNotNull(SwerveConstants.kSwerveKinematics);
    }

    @Test
    void extensionGearRatioIsOneToOne() {
        assertEquals(1.0, IntakeConstants.kExtensionGearRatio, 1e-9);
    }

    @Test
    void allCanIdsAreUniqueOnCANivoreBus() {
        Set<Integer> canivoreIds = new HashSet<>();
        assertTrue(canivoreIds.add(SwerveConstants.kFLDriveMotorID));
        assertTrue(canivoreIds.add(SwerveConstants.kFRDriveMotorID));
        assertTrue(canivoreIds.add(SwerveConstants.kBLDriveMotorID));
        assertTrue(canivoreIds.add(SwerveConstants.kBRDriveMotorID));
        assertTrue(canivoreIds.add(SwerveConstants.kFLSteerMotorID));
        assertTrue(canivoreIds.add(SwerveConstants.kFRSteerMotorID));
        assertTrue(canivoreIds.add(SwerveConstants.kBLSteerMotorID));
        assertTrue(canivoreIds.add(SwerveConstants.kBRSteerMotorID));
        assertTrue(canivoreIds.add(SwerveConstants.kFLCANcoderID));
        assertTrue(canivoreIds.add(SwerveConstants.kFRCANcoderID));
        assertTrue(canivoreIds.add(SwerveConstants.kBLCANcoderID));
        assertTrue(canivoreIds.add(SwerveConstants.kBRCANcoderID));
        assertTrue(canivoreIds.add(SwerveConstants.kPigeonID));
    }

    @Test
    void allCanIdsAreUniqueOnRioBus() {
        Set<Integer> rioIds = new HashSet<>();
        assertTrue(rioIds.add(IntakeConstants.kExtensionMotorID));
        assertTrue(rioIds.add(IntakeConstants.kRollerMotorID));
        assertTrue(rioIds.add(IntakeConstants.kEntrySensorID));
        assertTrue(rioIds.add(IntakeConstants.kMiddleSensorID));
        assertTrue(rioIds.add(IntakeConstants.kExitSensorID));
        assertTrue(rioIds.add(ShooterConstants.kAgitatorMotorID));
        assertTrue(rioIds.add(ShooterConstants.kKickerMotorID));
        assertTrue(rioIds.add(ShooterConstants.kPreShooterMotorID));
        assertTrue(rioIds.add(ShooterConstants.kShooterPrimaryMotorID));
        assertTrue(rioIds.add(ShooterConstants.kShooterSecondaryMotorID));
    }
}
