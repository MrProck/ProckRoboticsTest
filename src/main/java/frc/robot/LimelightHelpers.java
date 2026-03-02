package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Helper class for interacting with a Limelight camera over NetworkTables.
 *
 * <p>Provides methods for reading AprilTag pose estimates, setting pipeline
 * indices, and supplying robot orientation for MegaTag2.
 *
 * <p>Adapted from the Limelight community LimelightHelpers library.
 */
public final class LimelightHelpers {

    private LimelightHelpers() {}

    // -----------------------------------------------------------------------
    // Data classes
    // -----------------------------------------------------------------------

    /**
     * Represents a pose estimate returned by the Limelight.
     */
    public static class PoseEstimate {
        /** Estimated field-relative robot pose. */
        public Pose2d pose;

        /** FPGA timestamp of the measurement (seconds). */
        public double timestampSeconds;

        /** Latency of the pipeline in milliseconds. */
        public double latency;

        /** Number of AprilTags used to compute the estimate. */
        public int tagCount;

        /** Average area of the detected tags (0-100% of image). */
        public double tagSpan;

        /** Average distance to the detected tags (meters). */
        public double avgTagDist;

        /** Average tag area in the image (0-100%). */
        public double avgTagArea;

        public PoseEstimate() {
            pose = new Pose2d();
            timestampSeconds = 0;
            latency = 0;
            tagCount = 0;
            tagSpan = 0;
            avgTagDist = 0;
            avgTagArea = 0;
        }
    }

    // -----------------------------------------------------------------------
    // NetworkTable helpers
    // -----------------------------------------------------------------------

    private static NetworkTable getLimelightTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    private static NetworkTableEntry getLimelightEntry(String tableName, String entryName) {
        return getLimelightTable(tableName).getEntry(entryName);
    }

    private static double getLimelightDouble(String tableName, String entryName) {
        return getLimelightEntry(tableName, entryName).getDouble(0.0);
    }

    private static double[] getLimelightDoubleArray(String tableName, String entryName) {
        return getLimelightEntry(tableName, entryName).getDoubleArray(new double[0]);
    }

    private static void setLimelightDouble(String tableName, String entryName, double value) {
        getLimelightEntry(tableName, entryName).setDouble(value);
    }

    private static void setLimelightDoubleArray(
            String tableName, String entryName, double[] values) {
        getLimelightEntry(tableName, entryName).setDoubleArray(values);
    }

    private static String sanitizeName(String name) {
        if (name == null || name.isEmpty()) {
            return "limelight";
        }
        return name;
    }

    // -----------------------------------------------------------------------
    // Basic getters
    // -----------------------------------------------------------------------

    /** Returns 1.0 if the Limelight has a valid target, 0.0 otherwise. */
    public static double getTV(String limelightName) {
        return getLimelightDouble(limelightName, "tv");
    }

    /** Returns the horizontal offset to the target in degrees. */
    public static double getTX(String limelightName) {
        return getLimelightDouble(limelightName, "tx");
    }

    /** Returns the vertical offset to the target in degrees. */
    public static double getTY(String limelightName) {
        return getLimelightDouble(limelightName, "ty");
    }

    /** Returns the target area (0-100% of image). */
    public static double getTA(String limelightName) {
        return getLimelightDouble(limelightName, "ta");
    }

    /** Returns the pipeline latency contribution in milliseconds. */
    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightDouble(limelightName, "tl");
    }

    /** Returns the capture latency in milliseconds. */
    public static double getLatency_Capture(String limelightName) {
        return getLimelightDouble(limelightName, "cl");
    }

    // -----------------------------------------------------------------------
    // Pipeline control
    // -----------------------------------------------------------------------

    /** Sets the active pipeline index (0-9). */
    public static void setPipelineIndex(String limelightName, int index) {
        setLimelightDouble(limelightName, "pipeline", index);
    }

    // -----------------------------------------------------------------------
    // Robot orientation (MegaTag2)
    // -----------------------------------------------------------------------

    /**
     * Supplies the robot's IMU orientation to the Limelight for MegaTag2
     * pose estimation.
     *
     * @param limelightName Limelight NetworkTables name.
     * @param yaw           Robot yaw in degrees (positive = CCW).
     * @param yawRate       Robot yaw rate in degrees per second.
     * @param pitch         Robot pitch in degrees.
     * @param pitchRate     Robot pitch rate in degrees per second.
     * @param roll          Robot roll in degrees.
     * @param rollRate      Robot roll rate in degrees per second.
     */
    public static void SetRobotOrientation(
            String limelightName,
            double yaw, double yawRate,
            double pitch, double pitchRate,
            double roll, double rollRate) {
        setLimelightDoubleArray(limelightName, "robot_orientation_set",
            new double[] {yaw, yawRate, pitch, pitchRate, roll, rollRate});
    }

    // -----------------------------------------------------------------------
    // Pose estimation
    // -----------------------------------------------------------------------

    /**
     * Parses a raw botpose array from the Limelight into a {@link Pose2d}.
     *
     * <p>The array layout is:
     * [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgTagDist,
     * avgTagArea, ...]
     *
     * @param rawPose The raw double array from the Limelight.
     * @return A Pose2d, or {@code new Pose2d()} if the array is too short.
     */
    private static Pose2d parsePose2d(double[] rawPose) {
        if (rawPose == null || rawPose.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(
            new Translation2d(rawPose[0], rawPose[1]),
            Rotation2d.fromDegrees(rawPose[5]));
    }

    /**
     * Parses a raw botpose array into a {@link PoseEstimate}, including tag
     * metadata and latency-corrected timestamps.
     *
     * @param rawPose       The raw double array from the Limelight.
     * @param limelightName The name of the Limelight (for capture latency lookup).
     * @return A populated PoseEstimate, or null if the data is invalid.
     */
    private static PoseEstimate parsePoseEstimate(double[] rawPose, String limelightName) {
        if (rawPose == null || rawPose.length < 11) {
            return null;
        }

        PoseEstimate estimate = new PoseEstimate();
        estimate.pose = parsePose2d(rawPose);

        double latencyMs = rawPose[6];
        estimate.latency = latencyMs;
        estimate.timestampSeconds =
            (edu.wpi.first.wpilibj.Timer.getFPGATimestamp())
            - (latencyMs / 1000.0)
            - (getLatency_Capture(limelightName) / 1000.0);

        estimate.tagCount   = (int) rawPose[7];
        estimate.tagSpan    = rawPose[8];
        estimate.avgTagDist = rawPose[9];
        estimate.avgTagArea = rawPose[10];

        return estimate;
    }

    /**
     * Returns the MegaTag1 (standard) WPILib Blue-alliance-origin pose estimate.
     *
     * @param limelightName Limelight NetworkTables name.
     * @return A PoseEstimate, or null if no valid data.
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        double[] rawPose = getLimelightDoubleArray(limelightName, "botpose_wpiblue");
        return parsePoseEstimate(rawPose, limelightName);
    }

    /**
     * Returns the MegaTag2 WPILib Blue-alliance-origin pose estimate.
     *
     * <p>MegaTag2 uses the robot's supplied IMU orientation to improve pose
     * accuracy, especially when only a single tag is visible.
     *
     * @param limelightName Limelight NetworkTables name.
     * @return A PoseEstimate, or null if no valid data.
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
        double[] rawPose = getLimelightDoubleArray(limelightName, "botpose_orb_wpiblue");
        return parsePoseEstimate(rawPose, limelightName);
    }

    /**
     * Returns the MegaTag1 WPILib Red-alliance-origin pose estimate.
     *
     * @param limelightName Limelight NetworkTables name.
     * @return A PoseEstimate, or null if no valid data.
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
        double[] rawPose = getLimelightDoubleArray(limelightName, "botpose_wpired");
        return parsePoseEstimate(rawPose, limelightName);
    }

    /**
     * Returns the MegaTag2 WPILib Red-alliance-origin pose estimate.
     *
     * @param limelightName Limelight NetworkTables name.
     * @return A PoseEstimate, or null if no valid data.
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String limelightName) {
        double[] rawPose = getLimelightDoubleArray(limelightName, "botpose_orb_wpired");
        return parsePoseEstimate(rawPose, limelightName);
    }
}
