package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    private Constants() {}

    public static final class SwerveConstants {
        public static final String kCANivoreName = "CANivore";
        public static final int kFLDriveMotorID = 1;
        public static final int kFRDriveMotorID = 3;
        public static final int kBLDriveMotorID = 5;
        public static final int kBRDriveMotorID = 7;
        public static final int kFLSteerMotorID = 2;
        public static final int kFRSteerMotorID = 4;
        public static final int kBLSteerMotorID = 6;
        public static final int kBRSteerMotorID = 8;
        public static final int kFLCANcoderID = 9;
        public static final int kFRCANcoderID = 10;
        public static final int kBLCANcoderID = 11;
        public static final int kBRCANcoderID = 12;
        public static final int kPigeonID = 13;
        public static final double kFLCANcoderOffset = 0.0;
        public static final double kFRCANcoderOffset = 0.0;
        public static final double kBLCANcoderOffset = 0.0;
        public static final double kBRCANcoderOffset = 0.0;
        public static final boolean kFLDriveInverted = true;
        public static final boolean kFRDriveInverted = false;
        public static final boolean kBLDriveInverted = true;
        public static final boolean kBRDriveInverted = false;
        public static final boolean kFLSteerInverted = true;
        public static final boolean kFRSteerInverted = true;
        public static final boolean kBLSteerInverted = true;
        public static final boolean kBRSteerInverted = true;
        public static final double kDriveGearRatio = 6.75;
        public static final double kSteerGearRatio = (150.0 / 7.0);
        public static final double kWheelDiameterMeters = 0.1016;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kTrackWidthMeters = 0.71;
        public static final double kWheelBaseMeters = 0.32;
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d( kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0),
            new Translation2d( kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0));
        public static final double kMaxDriveSpeedMetersPerSecond = 4.96;
        public static final double kMaxAngularVelocityRadiansPerSecond = 2.0 * Math.PI;
        public static final double kTeleopMaxDriveSpeed = 1.0;
        public static final double kTeleopMaxAngularSpeed = 0.75;
        public static final int kDriveCurrentLimitAmps = 100;
        public static final int kSteerCurrentLimitAmps = 40;
        public static final double kDriveP = 0.1;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveFF = 1.0 / (kMaxDriveSpeedMetersPerSecond / kWheelCircumferenceMeters);
        public static final double kSteerP = 7.0;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.0;
        public static final double kSteerFF = 0.0;
        public static final double kDeadband = 0.05;
    }

    public static final class IntakeConstants {
        public static final int     kExtensionMotorID          = 14;
        public static final int     kExtensionCurrentLimitAmps = 25;
        public static final boolean kExtensionMotorInverted    = false;
        public static final double kExtensionP  = 0.1;
        public static final double kExtensionI  = 0.0;
        public static final double kExtensionD  = 0.0;
        public static final double kExtensionFF = 0.0;
        public static final double kExtensionRetractedPosition = 0.0;
        public static final double kExtensionExtendedPosition  = 50.0;
        public static final int     kRollerMotorID          = 15;
        public static final int     kRollerCurrentLimitAmps = 17;
        public static final boolean kRollerMotorInverted    = false;
        public static final double kRollerForwardSpeed  = 1.0;
        public static final double kRollerReverseSpeed  = -0.5;
        // Right trigger activation threshold (0.0-1.0)
        public static final double kTriggerThreshold    = 0.1;
        // AndyMark am-5636 CAN color sensors (RIO CAN bus)
        public static final int kEntrySensorID  = 16;
        public static final int kMiddleSensorID = 17;
        public static final int kExitSensorID   = 18;
    }

    public static final class ShooterConstants {
        // CAN IDs (RIO CAN bus)
        public static final int kAgitatorMotorID          = 19;
        public static final int kKickerMotorID            = 20;
        public static final int kPreShooterMotorID        = 21;
        public static final int kShooterPrimaryMotorID    = 22;
        public static final int kShooterSecondaryMotorID  = 23;

        // Current limits (amps)
        public static final int kAgitatorCurrentLimitAmps          = 20;
        public static final int kKickerCurrentLimitAmps            = 40;
        public static final int kPreShooterCurrentLimitAmps        = 40;
        public static final int kShooterCurrentLimitAmps           = 140;

        // Motor inversion
        public static final boolean kAgitatorInverted          = false;
        public static final boolean kKickerInverted            = false;
        public static final boolean kPreShooterInverted        = false;
        public static final boolean kShooterPrimaryInverted    = false;
        public static final boolean kShooterSecondaryInverted  = true;

        // --- Agitator (NEO + SparkMax) ---
        public static final double kAgitatorForwardRPM  = 3000.0;
        public static final double kAgitatorReverseRPM  = 1700.0;
        public static final double kAgitatorP           = 0.0002;
        public static final double kAgitatorI           = 0.0;
        public static final double kAgitatorD           = 0.0;
        public static final double kAgitatorFF          = 1.0 / 5676.0;  // NEO free speed

        // --- Kicker (NEO + SparkMax) ---
        public static final double kKickerForwardRPM    = 3000.0;
        public static final double kKickerReverseRPM    = 1000.0;
        public static final double kKickerP             = 0.0002;
        public static final double kKickerI             = 0.0;
        public static final double kKickerD             = 0.0;
        public static final double kKickerFF            = 1.0 / 5676.0;  // NEO free speed

        // --- Pre-Shooter (NEO Vortex + SparkFlex) ---
        public static final double kPreShooterForwardRPM = 5000.0;
        public static final double kPreShooterReverseRPM = 1000.0;
        public static final double kPreShooterP          = 0.0002;
        public static final double kPreShooterI          = 0.0;
        public static final double kPreShooterD          = 0.0;
        public static final double kPreShooterFF         = 1.0 / 6784.0;  // NEO Vortex free speed

        // --- Shooter (2x NEO Vortex + SparkFlex) ---
        public static final double kShooterForwardRPM    = 5500.0;
        public static final double kShooterReverseRPM    = 500.0;
        public static final double kShooterP             = 0.0002;
        public static final double kShooterI             = 0.0;
        public static final double kShooterD             = 0.0;
        public static final double kShooterFF            = 1.0 / 6784.0;  // NEO Vortex free speed

        // Shooter flywheel speed detection (used by ShootCommand to gate feeding)
        public static final double kShooterTargetRPM         = 5500.0;
        public static final double kShooterSpeedToleranceRPM = 200.0;
    }

    public static final class VisionConstants {
        /** NetworkTables name of the Limelight 4 camera. */
        public static final String kLimelightName = "limelight";

        /**
         * Maximum acceptable distance (meters) between the vision estimate and the
         * current odometry pose. Measurements farther away are rejected to avoid
         * teleporting the robot.
         */
        public static final double kMaxAcceptableDistanceMeters = 1.5;

        /**
         * Maximum acceptable rotation error (degrees) between the vision heading
         * and the gyro heading. Measurements with larger heading differences are
         * rejected.
         */
        public static final double kMaxAcceptableRotationDegrees = 15.0;

        /**
         * Minimum number of AprilTag targets the Limelight must see for the pose
         * estimate to be trusted.
         */
        public static final int kMinTagCount = 1;

        /**
         * Standard deviations for single-tag vision measurements
         * (x meters, y meters, heading radians).
         */
        public static final double kSingleTagStdDevX = 0.9;
        public static final double kSingleTagStdDevY = 0.9;
        public static final double kSingleTagStdDevTheta = 999.0;

        /**
         * Standard deviations for multi-tag vision measurements
         * (x meters, y meters, heading radians).
         */
        public static final double kMultiTagStdDevX = 0.5;
        public static final double kMultiTagStdDevY = 0.5;
        public static final double kMultiTagStdDevTheta = 999.0;
    }

    public static final class OIConstants {
        public static final int    kDriverControllerPort   = 0;
        public static final int    kOperatorControllerPort = 1;
        public static final double kDriveDeadband          = 0.05;
    }

    public static final class AutoConstants {
        public static final double kAutoDriveDistanceMeters = 2.0;
        public static final double kAutoDriveSpeed          = 0.5;
        public static final double kAutoDriveTimeSeconds    = 2.0;
    }

    public static final class PathPlannerConstants {
        public static final double kTranslationP = 5.0;
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;

        public static final double kRotationP = 5.0;
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;

        /** Distance from robot center to a swerve module (meters). */
        public static final double kDriveBaseRadius = Math.hypot(
            SwerveConstants.kWheelBaseMeters / 2.0,
            SwerveConstants.kTrackWidthMeters / 2.0);
    }
}
