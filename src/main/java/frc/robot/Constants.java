package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    private Constants() {}

    /**
     * Constants for the swerve drive system including CAN IDs, gear ratios, kinematics, PID gains,
     * and speed limits for SDS MK4 L2 modules with Falcon 500 motors on a CANivore bus.
     */
    public static final class SwerveConstants {
        public static final String kCANivoreBus = "CANivore";
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
        public static final double kFLCANcoderOffset =  0.4736328125;
        public static final double kFRCANcoderOffset = -0.484375;
        public static final double kBLCANcoderOffset = -0.274169921875;
        public static final double kBRCANcoderOffset =  0.498046875;
        public static final boolean kFLDriveInverted = false;
        public static final boolean kFRDriveInverted = true;
        public static final boolean kBLDriveInverted = false;
        public static final boolean kBRDriveInverted = true;
        public static final boolean kFLSteerInverted = false;
        public static final boolean kFRSteerInverted = false;
        public static final boolean kBLSteerInverted = false;
        public static final boolean kBRSteerInverted = false;
        public static final double kDriveGearRatio = 6.746031746031747;
        public static final double kSteerGearRatio = 12.8;
        public static final double kWheelDiameterMeters = 0.1016;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kTrackWidthMeters = 0.6985;  // 27.5 inches
        public static final double kWheelBaseMeters  = 0.3175;  // 12.5 inches
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d( kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0),
            new Translation2d( kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0));
        public static final double kMaxDriveSpeedMetersPerSecond = 4.58;
        public static final double kMaxAngularVelocityRadiansPerSecond = 2.0 * Math.PI;
        public static final double kTeleopMaxDriveSpeed = 1.0;
        public static final double kTeleopMaxAngularSpeed = 0.75;
        public static final int kDriveCurrentLimitAmps = 60;
        public static final int kSteerCurrentLimitAmps = 40;
        public static final double kDriveP  = 0.1;
        public static final double kDriveI  = 0.0;
        public static final double kDriveD  = 0.0;
        public static final double kDriveFF = 0.124 * kDriveGearRatio;   // kV scaled from motor rot/s to wheel rot/s
        public static final double kSteerP  = 100.0;
        public static final double kSteerI  = 0.0;
        public static final double kSteerD  = 0.5;
        public static final double kSteerS  = 0.1;     // static friction feedforward (V)
        public static final double kSteerFF = 1.59;    // kV (V per rot/s)
        public static final double kDeadband = 0.05;  // reduced: blended cubic curve handles near-zero sensitivity

        /**
         * Slew rate limit (units per second) applied to each drive axis in TeleopDriveCommand.
         * Limits how fast the commanded speed can change per second, smoothing acceleration and
         * preventing wheel slip. Higher values = snappier acceleration; lower = smoother ramp.
         */
        public static final double kTeleopSlewRatePerSecond = 2.5;

        /**
         * Max rate of change for the X-axis throttle multiplier (units/sec).
         * A value of 1.5 means it takes ~0.67s to go from stopped to full speed on the X axis.
         * Limits forward/backward acceleration to prevent tipping on the narrow wheelbase (12.5").
         * Y-axis (strafe) and rotation use the unramped throttle for instant responsiveness.
         */
        public static final double kThrottleXSlewRatePerSecond = 1.5;

        /**
         * Linear blend fraction for the input curve in TeleopDriveCommand.
         * Output = linearBlend * x + (1 - linearBlend) * x³
         * 0.0 = pure cubic (max sensitivity reduction near center)
         * 1.0 = pure linear (no curve)
         * 0.15 is a good starting point: smooth near zero, full speed at full stick.
         */
        public static final double kInputCurveLinearBlend = 1.0;  // fully linear: joystick doesn't reach 1.0 so cubic hurts top speed

        /**
         * When true, the Pigeon 2 yaw is negated before being used as the robot heading.
         * Set to true to match the WPILib convention (CCW positive) when the Pigeon reports CW positive.
         */
        public static final boolean kInvertGyro = false;

        /**
         * Physical offset of the Pigeon 2.0 from the robot's center of rotation (meters).
         * The Pigeon is mounted 6 inches (0.1524 m) to the right of center on the Y axis
         * (negative Y = right per WPILib convention where positive Y = left).
         *
         * <p>This does NOT affect yaw/heading accuracy — angular velocity is uniform across
         * the entire rigid robot body. However, if the Pigeon's accelerometer data is ever
         * used (e.g. for translational velocity estimation), this offset would need to be
         * compensated for by subtracting the centripetal acceleration term:
         * <pre>  a_center = a_pigeon - ω² × r</pre>
         * where ω is angular velocity (rad/s) and r is this offset distance (meters).
         */
        public static final double kPigeonOffsetXMeters = 0.0;     // forward/back from center (meters), positive = forward
        public static final double kPigeonOffsetYMeters = -0.1524; // left/right from center (meters), negative = right
    }

    /**
     * Constants for the intake/hopper subsystem including extension arm and roller motor CAN IDs,
     * PID gains, positions, current limits, and color sensor CAN IDs.
     *
     * <p>The extension arm is a spring-loaded hopper controlled by a NEO + gearbox (SparkMax, CAN 14).
     * Soft limits are set to ±192 rotations: +192 = fully extended (spring let out),
     * -192 = fully retracted (spring pulled back). Encoder is zeroed at the neutral/home position on boot.
     */
    public static final class IntakeConstants {
        public static final int     kExtensionMotorID          = 14;
        public static final int     kExtensionCurrentLimitAmps = 17;
        public static final boolean kExtensionMotorInverted    = false;
        public static final double kExtensionP  = 0.1;
        public static final double kExtensionI  = 0.0;
        public static final double kExtensionD  = 0.0;
        public static final double kExtensionFF = 0.0;
        /** Fully retracted position (rotations) — spring pulled back. */
        public static final double kExtensionRetractedPosition = -192.0;
        /** Fully extended position (rotations) — spring let out. */
        public static final double kExtensionExtendedPosition  =  192.0;
        /** Gear ratio between motor and extension output. */
        public static final double kExtensionGearRatio = 1.0; // Confirmed 1:1 gear ratio
        /** Conversion factor: output rotations per motor rotation. */
        public static final double kExtensionPositionConversionFactor = 1.0 / kExtensionGearRatio;
        public static final int     kRollerMotorID          = 15;
        public static final int     kRollerCurrentLimitAmps = 25;
        public static final boolean kRollerMotorInverted    = false; // reverted: motor runs correct direction uninverted
        public static final double kRollerForwardSpeed  = 1.0;
        public static final double kRollerReverseSpeed  = -0.5;
        // AndyMark am-5636 CAN color sensors (RIO CAN bus)
        public static final int kEntrySensorID  = 16;
        public static final int kMiddleSensorID = 17;
        public static final int kExitSensorID   = 18;
    }

    /**
     * Constants for the 4-stage shooter pipeline (agitator, kicker, pre-shooter, dual shooter flywheels)
     * including CAN IDs, current limits, PID gains, RPM targets, and spin-up timeout.
     */
    public static final class ShooterConstants {
        // CAN IDs (RIO CAN bus)
        public static final int kAgitatorMotorID          = 19;
        public static final int kKickerMotorID            = 20;
        public static final int kPreShooterMotorID        = 21;
        public static final int kShooterPrimaryMotorID    = 22;
        public static final int kShooterSecondaryMotorID  = 23;

        // Current limits (amps)
        public static final int kAgitatorCurrentLimitAmps          = 30;
        public static final int kKickerCurrentLimitAmps            = 50;
        public static final int kPreShooterCurrentLimitAmps        = 40;
        /** Current limit for each shooter flywheel motor (amps). */
        public static final int kShooterCurrentLimitAmps = 70;

        // Motor inversion
        public static final boolean kAgitatorInverted          = true;
        public static final boolean kKickerInverted            = false;
        public static final boolean kPreShooterInverted        = true;
        public static final boolean kShooterPrimaryInverted    = false;
        public static final boolean kShooterSecondaryInverted  = true;

        // --- Agitator (NEO + SparkMax) ---
        public static final double kAgitatorForwardRPM  = 3000.0;
        public static final double kAgitatorReverseRPM  = 1700.0;
        public static final double kAgitatorIntakeRPM   = 2000.0;
        public static final double kAgitatorP           = 0.0002;
        public static final double kAgitatorI           = 0.0;
        public static final double kAgitatorD           = 0.0;
        public static final double kAgitatorFF          = 1.0 / 5676.0;  // NEO free speed

        // --- Kicker (NEO Vortex + SparkFlex) ---
        public static final double kKickerForwardRPM    = 3000.0;
        public static final double kKickerReverseRPM    = 1000.0;
        /** Slow reverse speed (RPM) used during shooter spin-up to prevent pre-loading. */
        public static final double kKickerSlowInvertRPM = 500.0;
        public static final double kKickerP             = 0.0002;
        public static final double kKickerI             = 0.0;
        public static final double kKickerD             = 0.0;
        public static final double kKickerFF            = 1.0 / 6784.0;  // NEO Vortex free speed

        // --- Pre-Shooter (NEO Vortex + SparkFlex) ---
        public static final double kPreShooterForwardRPM = 5000.0;
        public static final double kPreShooterReverseRPM = 1000.0;
        // FUTURE TUNING: Increasing kPreShooterP (e.g. 0.0004) will make the pre-shooter
        // ramp up faster. Watch for overshoot/oscillation on the SmartDashboard RPM readout.
        public static final double kPreShooterP          = 0.0002;
        public static final double kPreShooterI          = 0.0;
        public static final double kPreShooterD          = 0.0;
        public static final double kPreShooterFF         = 1.0 / 6784.0;  // NEO Vortex free speed

        // --- Shooter (2x NEO Vortex + SparkFlex) ---
        public static final double kShooterForwardRPM    = 5000.0;
        public static final double kShooterReverseRPM    = 500.0;
        public static final double kReverseAllDutyCycle  = 0.3;  // 30% duty cycle for reverse (bang-bang motors)
        // FUTURE TUNING: To improve shooter spin-up time, consider:
        //   1. Increase kShooterP (e.g. 0.0004) — more aggressive PID response to large RPM errors.
        //      Watch SmartDashboard "Shooter/Shooter Primary RPM" for overshoot/oscillation.
        //   2. Fine-tune kShooterFF (e.g. 1.0 / 6500.0) — slightly higher FF gives a bigger
        //      initial voltage push toward setpoint.
        //   3. Add .closedLoopRampRate(0.0) to the SparkFlex config in ShooterSubsystem.java
        //      to explicitly eliminate any ramp delay on the controller side.
        //   4. Widen kShooterSpeedToleranceRPM (e.g. 300.0) to reduce the perceived wait time
        //      before the ShootCommand gates feeding — only if shot accuracy allows it.
        public static final double kShooterP             = 0.001;  // previous PID P gain (unused, kept for reference)
        /** High P gain for bang-bang style onboard PID (1kHz on motor controller). Full power at >=10 RPM error. */
        public static final double kShooterBangBangP    = 0.1;
        public static final double kShooterI             = 0.0;
        public static final double kShooterD             = 0.0;
        public static final double kShooterFF            = 1.0 / 6784.0;  // NEO Vortex free speed

        // Shooter flywheel speed detection (used by ShootCommand to gate feeding)
        public static final double kShooterSpeedToleranceRPM    = 175.0;
        public static final double kPreShooterSpeedToleranceRPM = 300.0;

        /** Pre-shooter RPM threshold at which the kicker begins feeding. */
        public static final double kPreShooterKickerThresholdRPM = 3000.0;

        /** The maximum time (seconds) to wait for shooter spin-up before feeding anyway. */
        public static final double kShooterSpinUpTimeoutSeconds = 3.0;

        /** Minimum time (seconds) to keep feeding after the feed gate opens (auto only). */
        public static final double kAutoFeedDurationSeconds = 0.5;

        /** RPM increment/decrement per D-pad press for manual RPM adjustment. */
        public static final double kManualShootRPMStep   = 50.0;

        /** Minimum RPM allowed via manual D-pad adjustment. */
        public static final double kManualShootMinRPM    = 500.0;

        /** Maximum RPM allowed via manual D-pad adjustment (operator cap). */
        public static final double kManualShootMaxRPM    = 3200.0;
    }

    /**
     * Constants for the Limelight 4 AprilTag vision system including outlier rejection thresholds
     * and measurement standard deviations.
     */
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

        // Camera mount position relative to robot center (meters and degrees)
        /** Forward offset of the camera from the robot center (meters, positive = forward). */
        public static final double kCameraForwardOffsetMeters = -0.1143; // 4.5 inches behind center
        /** Side offset of the camera from the robot center (meters, positive = left). */
        public static final double kCameraSideOffsetMeters    =  0.3175; // 12.5 inches to the left (positive = left)
        /** Height of the camera above the floor (meters). */
        public static final double kCameraUpOffsetMeters      =  0.5207; // 20.5 inches from floor
        /** Camera roll angle (degrees). */
        public static final double kCameraRollDegrees         =  0.0;
        /** Camera pitch angle (degrees, positive = tilted up). */
        public static final double kCameraPitchDegrees        =  20.0;  // camera is mounted at 20 degrees
        /** Camera yaw angle (degrees, positive = rotated left). */
        public static final double kCameraYawDegrees          =  0.0;
    }

    /**
     * Operator Interface constants including controller ports, deadband, and trigger thresholds.
     */
    public static final class OIConstants {
        public static final int    kDriverControllerPort   = 0;
        public static final int    kOperatorControllerPort = 1;
        public static final double kDriveDeadband          = 0.05;
        // Trigger activation threshold (0.0-1.0)
        public static final double kTriggerThreshold       = 0.1;
    }

    /**
     * Constants for the basic timed autonomous routine.
     */
    public static final class AutoConstants {
        public static final double kAutoDriveDistanceMeters = 2.0;
        public static final double kAutoDriveSpeed          = 0.5;
        public static final double kAutoDriveTimeSeconds    = 2.0;
    }

    /**
     * PID constants and geometry for PathPlanner autonomous path following.
     */
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

    /**
     * Constants for the Limelight orbital drive mode targeting the 2026 REBUILT field hub.
     *
     * <p>The hub is a central hexagonal field element. Its center coordinates are derived
     * from the official WPILib 2026-rebuilt-welded AprilTag field layout JSON.
     * Field dimensions: 16.541 m (length) × 8.069 m (width).
     *
     * <p>Blue hub AprilTag cluster (IDs 18–21, 24–27) center: (~4.876, ~4.035)
     * Red  hub AprilTag cluster (IDs  2–5,  8–11) center: (~11.916, ~4.035)
     */
    public static final class OrbitalConstants {

        // ---- Hub positions (meters, WPILib Blue-alliance origin) ----

        /** X coordinate of the Blue alliance hub center (meters). */
        public static final double kHubBlueX = 4.8755;

        /** X coordinate of the Red alliance hub center (meters). */
        public static final double kHubRedX  = 11.9155;

        /**
         * Y coordinate of the hub center shared by both alliances (meters).
         * The hub sits on the field centerline.
         */
        public static final double kHubY = 4.0346;

        // ---- Rotational PID (heading lock toward hub) ----

        /** Proportional gain for the heading-to-hub PID controller. */
        public static final double kOrbitalP = 5.5;

        /** Integral gain for the heading-to-hub PID controller. */
        public static final double kOrbitalI = 0.0;

        /** Derivative gain for the heading-to-hub PID controller. */
        public static final double kOrbitalD = 0.2;

        /**
         * Heading tolerance (degrees). The PID output is zeroed when the robot
         * is within this angle of the hub direction.
         */
        public static final double kOrbitalToleranceDegrees = 2.0;

        /**
         * Maximum rotational output magnitude (rad/s) while orbiting.
         * Prevents the robot from spinning uncontrollably if heading error is large.
         */
        public static final double kOrbitalMaxRotationRadPerSec = Math.PI; // 180 °/s

        // ---- Orbital radius control ----

        /** Default orbit radius when the command first activates (meters). */
        public static final double kOrbitDefaultRadiusMeters = 2.5;
        /** Minimum allowed orbit radius (meters). */
        public static final double kOrbitMinRadiusMeters = 1.5;
        /** Maximum allowed orbit radius (meters). */
        public static final double kOrbitMaxRadiusMeters = 4.5;
        /** How fast the radius changes when the driver pushes the stick in/out (m/s). */
        public static final double kOrbitRadiusAdjustRate = 1.0;
        /** Maximum tangential (orbital) speed (m/s). */
        public static final double kOrbitMaxTangentialSpeedMPS = 2.5;
        /** P gain for maintaining target orbit radius. */
        public static final double kOrbitRadialP = 2.0;
    }

    /**
     * Distance-to-RPM interpolation table for vision-assisted shooting.
     *
     * <p>Each row is { distanceMeters, shooterRPM, preShooterRPM }.
     * Rows MUST be sorted by ascending distance.
     * Values between rows are linearly interpolated.
     * Values outside the range clamp to the nearest endpoint.
     *
     * <p>TUNING GUIDE:
     *   1. Drive the robot to a known distance from the hub center.
     *   2. Use SmartDashboard "Shooter/Distance To Hub" to confirm the reading.
     *   3. Manually tune RPMs until shots are consistent at that distance.
     *   4. Record those RPMs as a row here.
     *   5. Repeat at several distances (e.g. 1.5m, 2.5m, 3.5m, 4.5m).
     *
     * <p>NOTE: Distances are from the robot center to the hub CENTER.
     * Hub radius (center to edge) is approximately 0.6096 m (24 in).
     * Measured distances from hub EDGE have had 0.6096 m added to convert to hub center.
     */
    public static final class ShooterTableConstants {

        /**
         * Interpolation table: each row is { distanceMeters, shooterRPM, preShooterRPM }.
         * Distances are in meters from the hub CENTER to the robot center.
         *
         * Tuned shooter RPM values (measured from hub edge, converted to hub center distance):
         *   3 ft from edge  (1.524 m from center) → 2600 RPM
         *   4 ft from edge  (1.829 m from center) → 2700 RPM
         *   5 ft from edge  (2.134 m from center) → 2725 RPM
         *   7'8" from edge  (2.946 m from center) → 3150 RPM
         *
         * Pre-shooter RPM values are not yet tuned — set equal to shooter RPM as a starting point.
         * *** PRE-SHOOTER RPM VALUES ARE PLACEHOLDERS — TUNE ON THE ACTUAL ROBOT ***
         */
        public static final double[][] kShooterTable = {
            // dist from hub center (m)   shooterRPM  preShooterRPM
            {  1.524,                      2600.0,     2600.0 },  // 3 ft from hub edge
            {  1.829,                      2700.0,     2700.0 },  // 4 ft from hub edge
            {  2.134,                      2725.0,     2725.0 },  // 5 ft from hub edge
            {  2.946,                      3150.0,     3150.0 },  // 7 ft 8 in from hub edge
        };

        /** Column index for distance in kShooterTable. */
        public static final int kColDistance      = 0;
        /** Column index for shooter flywheel RPM in kShooterTable. */
        public static final int kColShooterRPM    = 1;
        /** Column index for pre-shooter RPM in kShooterTable. */
        public static final int kColPreShooterRPM = 2;

        /**
         * Fallback shooter RPM used when no vision target is available.
         * Should match a mid-range shot — tune to your most common shooting distance.
         */
        public static final double kFallbackShooterRPM    = 3150.0;
        public static final double kFallbackPreShooterRPM = ShooterConstants.kPreShooterForwardRPM;
    }
}