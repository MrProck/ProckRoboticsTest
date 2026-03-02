package frc.robot;

public final class Constants {

    private Constants() {}

    // =========================================================
    // Swerve Drive CAN IDs (CTRE Phoenix 6 — TalonFX & CANcoder)
    // =========================================================
    public static final class SwerveConstants {
        // Drive Motors (TalonFX / Falcon 500)
        public static final int kFLDriveMotorID  = 1;
        public static final int kFRDriveMotorID  = 3;
        public static final int kBLDriveMotorID  = 5;
        public static final int kBRDriveMotorID  = 7;

        // Steer Motors (TalonFX / Falcon 500)
        public static final int kFLSteerMotorID  = 2;
        public static final int kFRSteerMotorID  = 4;
        public static final int kBLSteerMotorID  = 6;
        public static final int kBRSteerMotorID  = 8;

        // CANcoders
        public static final int kFLCANcoderID    = 9;
        public static final int kFRCANcoderID    = 10;
        public static final int kBLCANcoderID    = 11;
        public static final int kBRCANcoderID    = 12;

        // Pigeon 2.0 IMU
        public static final int kPigeonID        = 13;

        // CANcoder offsets (rotations) — tune these with Tuner X
        public static final double kFLCANcoderOffset = 0.0;
        public static final double kFRCANcoderOffset = 0.0;
        public static final double kBLCANcoderOffset = 0.0;
        public static final double kBRCANcoderOffset = 0.0;

        // SDS MK4 L2 — Drive gear ratio: 6.75:1, Steer gear ratio: 150/7:1
        public static final double kDriveGearRatio  = 6.75;
        public static final double kSteerGearRatio  = (150.0 / 7.0);
        public static final double kWheelDiameterMeters = 0.1016; // 4 inches

        // Robot geometry (meters) — measure your robot!
        public static final double kTrackWidthMeters  = 0.5588; // placeholder ~22 in
        public static final double kWheelBaseMeters   = 0.5588; // placeholder ~22 in

        // Max speeds
        public static final double kMaxDriveSpeedMetersPerSecond    = 4.96; // MK4 L2 theoretical max
        public static final double kMaxSteerAngularVelocityRadPerSec = 2.0 * Math.PI;

        // Drive current limit (amps)
        public static final int kDriveCurrentLimitAmps = 100;

        // Deadband
        public static final double kDeadband = 0.05;
    }

    // =========================================================
    // Intake CAN IDs (REV)
    // =========================================================
    public static final class IntakeConstants {
        // Part 1 — Extension (NEO + SparkMax)
        public static final int kExtensionMotorID          = 14;
        public static final int kExtensionCurrentLimitAmps = 25;

        // PID gains for extension position control (tune on robot)
        public static final double kExtensionP              = 0.1;
        public static final double kExtensionI              = 0.0;
        public static final double kExtensionD              = 0.0;
        public static final double kExtensionFF             = 0.0;

        // Extension positions (rotations) — tune on robot
        public static final double kExtensionRetractedPosition = 0.0;
        public static final double kExtensionExtendedPosition  = 50.0;

        // Part 2 — Roller (NEO Vortex + SparkFlex)
        public static final int kRollerMotorID             = 15;
        public static final int kRollerCurrentLimitAmps    = 17;

        // Roller speeds (RPM)
        public static final double kRollerForwardRPM       = 4500.0;
        public static final double kRollerReverseRPM       = -2500.0;
        public static final double kRollerStopRPM          = 0.0;
    }

    // =========================================================
    // Operator Interface
    // =========================================================
    public static final class OIConstants {
        public static final int kDriverControllerPort   = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDriveDeadband       = 0.05;
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