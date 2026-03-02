package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    private Constants() {}

    // =========================================================
    // Swerve Drive Constants (CTRE Phoenix 6 — TalonFX & CANcoder)
    // =========================================================
    public static final class SwerveConstants {
        public static final String kCANivoreName = "CANivore";

        // Drive Motors (TalonFX / Falcon 500)
        public static final int kFLDriveMotorID = 1;
        public static final int kFRDriveMotorID = 3;
        public static final int kBLDriveMotorID = 5;
        public static final int kBRDriveMotorID = 7;

        // Steer Motors (TalonFX / Falcon 500)
        public static final int kFLSteerMotorID = 2;
        public static final int kFRSteerMotorID = 4;
        public static final int kBLSteerMotorID = 6;
        public static final int kBRSteerMotorID = 8;

        // CANcoders
        public static final int kFLCANcoderID = 9;
        public static final int kFRCANcoderID = 10;
        public static final int kBLCANcoderID = 11;
        public static final int kBRCANcoderID = 12;

        // Pigeon 2.0 IMU
        public static final int kPigeonID = 13;

        // CANcoder offsets (rotations) — calibrate with Tuner X
        public static final double kFLCANcoderOffset = 0.0;
        public static final double kFRCANcoderOffset = 0.0;
        public static final double kBLCANcoderOffset = 0.0;
        public static final double kBRCANcoderOffset = 0.0;

        // Motor inversions — SDS MK4 default wiring (all non-inverted)
        public static final boolean kFLDriveInverted = false;
        public static final boolean kFRDriveInverted = false;
        public static final boolean kBLDriveInverted = false;
        public static final boolean kBRDriveInverted = false;

        // Steer motors inverted per SDS MK4 default wiring
        public static final boolean kFLSteerInverted = true;
        public static final boolean kFRSteerInverted = true;
        public static final boolean kBLSteerInverted = true;
        public static final boolean kBRSteerInverted = true;

        // SDS MK4 L2 gear ratios
        public static final double kDriveGearRatio       = 6.75;
        public static final double kSteerGearRatio       = (150.0 / 7.0); // ~21.43:1
        public static final double kWheelDiameterMeters  = 0.1016;        // 4 inches
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        // Robot geometry — track width 71cm, wheelbase 32cm
        public static final double kTrackWidthMeters = 0.71;
        public static final double kWheelBaseMeters  = 0.32;

        // Swerve kinematics — module positions relative to robot center
        // Order: FL, FR, BL, BR
        public static final SwerveDriveKinematics kSwerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d( kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0),  // FL
                new Translation2d( kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),  // FR
                new Translation2d(-kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0),  // BL
                new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0)   // BR
            );

        // Max speeds — SDS MK4 L2 theoretical max ~4.96 m/s
        public static final double kMaxDriveSpeedMetersPerSecond     = 4.96;
        public static final double kMaxAngularVelocityRadiansPerSecond = 2.0 * Math.PI;

        // Teleop speed scaling
        public static final double kTeleopMaxDriveSpeed   = 1.0; // percent of max
        public static final double kTeleopMaxAngularSpeed = 0.75;

        // Drive current limit
        public static final int kDriveCurrentLimitAmps = 100;
        public static final int kSteerCurrentLimitAmps = 40;

        // Drive PID (velocity control)
        public static final double kDriveP  = 0.1;
        public static final double kDriveI  = 0.0;
        public static final double kDriveD  = 0.0;
        public static final double kDriveFF = 0.0;

        // Steer PID (position control)
        public static final double kSteerP  = 7.0;
        public static final double kSteerI  = 0.0;
        public static final double kSteerD  = 0.0;
        public static final double kSteerFF = 0.0;

        // Encoder conversion factors
        // Drive: motor rotations -> meters
        public static final double kDrivePositionConversionFactor =
            kWheelCircumferenceMeters / kDriveGearRatio;
        // Drive: motor RPM -> m/s
        public static final double kDriveVelocityConversionFactor =
            kDrivePositionConversionFactor / 60.0;

        // Input deadband
        public static final double kDeadband = 0.05;
    }

    // =========================================================
    // Intake Constants (REV)
    // =========================================================
    public static final class IntakeConstants {
        // Part 1 — Extension (NEO + SparkMax)
        public static final int    kExtensionMotorID             = 14;
        public static final int    kExtensionCurrentLimitAmps    = 25;
        public static final boolean kExtensionMotorInverted      = false;

        // PID gains for extension position control (tune on robot)
        public static final double kExtensionP  = 0.1;
        public static final double kExtensionI  = 0.0;
        public static final double kExtensionD  = 0.0;
        public static final double kExtensionFF = 0.0;

        // Extension positions (rotations) — tune on robot
        public static final double kExtensionRetractedPosition = 0.0;
        public static final double kExtensionExtendedPosition  = 50.0;

        // Part 2 — Roller (NEO Vortex + SparkFlex)
        public static final int    kRollerMotorID          = 15;
        public static final int    kRollerCurrentLimitAmps = 17;
        public static final boolean kRollerMotorInverted   = false;

        // Roller speeds (RPM)
        public static final double kRollerForwardRPM = 4500.0;
        public static final double kRollerReverseRPM = -2500.0;
        public static final double kRollerStopRPM    = 0.0;
    }

    // =========================================================
    // Operator Interface
    // =========================================================
    public static final class OIConstants {
        public static final int    kDriverControllerPort   = 0;
        public static final int    kOperatorControllerPort = 1;
        public static final double kDriveDeadband          = 0.05;
    }

    // =========================================================
    // Autonomous
    // =========================================================
    public static final class AutoConstants {
        public static final double kAutoDriveDistanceMeters = 2.0;
        public static final double kAutoDriveSpeed          = 0.5;
        public static final double kAutoDriveTimeSeconds    = 2.0;
    }
}