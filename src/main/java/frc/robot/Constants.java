package frc.robot;

public final class Constants {

    private Constants() {}

    public static final class DriveConstants {
        // CAN IDs — update these to match your robot's wiring
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        public static final double kMaxOutput = 1.0;
        public static final double kDeadband = 0.05;

        // Drive characterization values (tune for your robot)
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        public static final double kTrackwidthMeters = 0.69;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kAutoDriveDistanceMeters = 2.0;
        public static final double kAutoDriveSpeed = 0.5;
        public static final double kAutoDriveTimeSeconds = 2.0;
    }
}
