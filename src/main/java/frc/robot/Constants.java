// Constants for robot control

package frc.robot;

public class Constants {

    // -------------------------------------------------------------------------
    // Robot-specific constants
    // -------------------------------------------------------------------------
    public static final double kBRCANcoderOffset = 0.498046875;

    // -------------------------------------------------------------------------
    // Drive Constants
    // -------------------------------------------------------------------------
    public static final class DriveConstants {
        // Swerve module CAN IDs
        public static final int kFLDriveMotorID = 1;
        public static final int kFLSteerMotorID = 2;
        public static final int kFLCANcoderID   = 3;

        public static final int kFRDriveMotorID = 4;
        public static final int kFRSteerMotorID = 5;
        public static final int kFRCANcoderID   = 6;

        public static final int kBLDriveMotorID = 7;
        public static final int kBLSteerMotorID = 8;
        public static final int kBLCANcoderID   = 9;

        public static final int kBRDriveMotorID = 10;
        public static final int kBRSteerMotorID = 11;
        public static final int kBRCANcoderID   = 12;

        // Chassis dimensions (meters)
        public static final double kTrackWidthMeters  = 0.5588;  // ~22"
        public static final double kWheelBaseMeters    = 0.3175;  // ~12.5"
        public static final double kWheelDiameterMeters = 0.1016; // ~4"

        // Drive motor gear ratio (motor rotations per wheel rotation)
        public static final double kDriveGearRatio = 6.75;

        // Max speeds
        public static final double kMaxDriveSpeedMetersPerSecond     = 4.5;
        public static final double kMaxAngularSpeedRadiansPerSecond  = 2.0 * Math.PI;
    }

    // -------------------------------------------------------------------------
    // Teleop Constants
    // -------------------------------------------------------------------------
    public static final class TeleopConstants {
        /** Joystick deadband. */
        public static final double kDeadband = 0.05;

        /** Blending factor for cubic input curve (1.0 = fully linear, 0.0 = fully cubic). */
        public static final double kInputCurveLinearBlend = 1.0;

        /**
         * Slew rate limit (units per second) applied to each drive axis in TeleopDriveCommand.
         * Limits how fast the commanded speed can change per second, smoothing acceleration and
         * preventing wheel slip. Higher values = snappier acceleration; lower = smoother ramp.
         */
        public static final double kTeleopSlewRatePerSecond = 2.5;

        /** Maximum teleop drive speed as a fraction of kMaxDriveSpeedMetersPerSecond. */
        public static final double kTeleopMaxDriveSpeed = 1.0;

        /** Maximum teleop rotation speed as a fraction of kMaxAngularSpeedRadiansPerSecond. */
        public static final double kTeleopMaxRotationSpeed = 1.0;

        /**
         * Max rate of change for the X-axis throttle multiplier (units/sec).
         * A value of 1.5 means it takes ~0.67s to go from stopped to full speed on the X axis.
         * Limits forward/backward acceleration to prevent tipping on the narrow wheelbase (12.5").
         * Y-axis (strafe) and rotation use the unramped throttle for instant responsiveness.
         */
        public static final double kThrottleXSlewRatePerSecond = 1.5;
    }

    // -------------------------------------------------------------------------
    // Shooter Constants
    // -------------------------------------------------------------------------
    public static final class ShooterConstants {
        // CAN IDs
        public static final int kAgitatorMotorID         = 19;
        public static final int kKickerMotorID           = 20;
        public static final int kPreShooterMotorID       = 21;
        public static final int kShooterPrimaryMotorID   = 22;
        public static final int kShooterSecondaryMotorID = 23;

        // Motor inversion
        public static final boolean kAgitatorInverted         = false;
        public static final boolean kKickerInverted           = false;
        public static final boolean kPreShooterInverted       = false;
        public static final boolean kShooterPrimaryInverted   = false;
        public static final boolean kShooterSecondaryInverted = true;

        // Current limits (amps)
        public static final int kAgitatorCurrentLimitAmps    = 30;
        public static final int kKickerCurrentLimitAmps      = 50;
        public static final int kPreShooterCurrentLimitAmps  = 40;
        public static final int kShooterCurrentLimitAmps     = 90;

        // --- Forward RPMs ---
        public static final double kAgitatorForwardRPM    = 3000.0;
        public static final double kKickerForwardRPM      = 4000.0;
        public static final double kPreShooterForwardRPM  = 5000.0;
        public static final double kShooterForwardRPM     = 5000.0;

        // --- Reverse RPMs (for clearing jams) ---
        public static final double kAgitatorReverseRPM    = 1500.0;
        public static final double kKickerReverseRPM      = 2000.0;
        public static final double kPreShooterReverseRPM  = 2500.0;
        public static final double kShooterReverseRPM     = 2500.0;

        // --- Kicker slow reverse (anti-feed during spin-up) ---
        public static final double kKickerSlowInvertRPM = 500.0;

        // --- Pre-shooter kicker threshold ---
        public static final double kPreShooterKickerThresholdRPM = 3000.0;

        // --- Speed tolerances ---
        public static final double kShooterSpeedToleranceRPM    = 200.0;
        public static final double kPreShooterSpeedToleranceRPM = 200.0;

        // --- Spin-up timeout ---
        /** The maximum time (seconds) to wait for shooter spin-up before feeding anyway. */
        public static final double kShooterSpinUpTimeoutSeconds = 3.0;

        // --- Agitator PID ---
        public static final double kAgitatorP  = 0.0001;
        public static final double kAgitatorI  = 0.0;
        public static final double kAgitatorD  = 0.0;
        public static final double kAgitatorFF = 0.000175;

        // --- Kicker PID ---
        public static final double kKickerP  = 0.0001;
        public static final double kKickerI  = 0.0;
        public static final double kKickerD  = 0.0;
        public static final double kKickerFF = 0.000155;

        // --- Pre-Shooter PID ---
        public static final double kPreShooterP  = 0.0001;
        public static final double kPreShooterI  = 0.0;
        public static final double kPreShooterD  = 0.0;
        public static final double kPreShooterFF = 0.000155;

        // --- Shooter PID ---
        public static final double kShooterP  = 0.0001;
        public static final double kShooterI  = 0.0;
        public static final double kShooterD  = 0.0;
        public static final double kShooterFF = 0.000155;
    }
}