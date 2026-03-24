package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OrbitalConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

import java.util.Optional;

/**
 * Vision subsystem using a Limelight 4 for AprilTag-based pose estimation.
 *
 * <p>Uses the Limelight's MegaTag2 pipeline to get field-relative robot poses
 * and feeds them into the {@link DriveSubsystem}'s {@code SwerveDrivePoseEstimator}.
 *
 * <p>Measurements are validated against the current odometry pose to reject outliers.
 */
public class VisionSubsystem extends SubsystemBase {

    private final DriveSubsystem m_driveSubsystem;
    private final String m_limelightName;

    private static final String kVerboseTelemetryKey = "Vision/Verbose Telemetry";

    private boolean m_hasTarget = false;
    private int m_tagCount = 0;
    private double m_distanceToHubMeters = -1.0;       // odometry-based, -1 = unknown
    private double m_directDistanceToHubMeters = -1.0; // Limelight tag-based, -1 = unknown

    // Blue hub AprilTag IDs: 18-21, 24-27  |  Red hub AprilTag IDs: 2-5, 8-11
    private static final int[] kBlueHubTagIds = { 18, 19, 20, 21, 24, 25, 26, 27 };
    private static final int[] kRedHubTagIds  = {  2,  3,  4,  5,  8,  9, 10, 11 };

    /**
     * Creates a new VisionSubsystem.
     *
     * @param driveSubsystem The drive subsystem to send vision poses to.
     */
    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_limelightName = VisionConstants.kLimelightName;
        Preferences.initBoolean(kVerboseTelemetryKey, false);

        // Configure the Limelight pipeline for AprilTag detection
        LimelightHelpers.setPipelineIndex(m_limelightName, 0);

        // Configure camera-to-robot transform so Limelight can compute field-relative poses
        LimelightHelpers.setCameraPose_RobotSpace(
            m_limelightName,
            VisionConstants.kCameraForwardOffsetMeters,
            VisionConstants.kCameraSideOffsetMeters,
            VisionConstants.kCameraUpOffsetMeters,
            VisionConstants.kCameraRollDegrees,
            VisionConstants.kCameraPitchDegrees,
            VisionConstants.kCameraYawDegrees);
    }

    /** Returns true if the Limelight currently sees at least one AprilTag. */
    public boolean hasTarget() {
        return m_hasTarget;
    }

    /** Returns the number of AprilTags currently detected. */
    public int getTagCount() {
        return m_tagCount;
    }

    /**
     * Returns the distance (meters) from the robot center to the alliance hub center,
     * computed from the fused odometry pose. Returns -1.0 if alliance is unknown.
     */
    public double getDistanceToHub() {
        return m_distanceToHubMeters;
    }

    /**
     * Returns the direct distance (meters) to the nearest visible hub AprilTag,
     * measured by the Limelight camera. Returns -1.0 if no hub tag is currently visible.
     *
     * <p>This is more accurate than the odometry-based distance for shooting because
     * it uses the raw camera measurement rather than the fused pose estimate.
     */
    public double getDirectDistanceToHub() {
        return m_directDistanceToHubMeters;
    }

    @Override
    public void periodic() {
        // Telemetry state for this cycle
        double lastPoseDiffMeters = 0.0;
        double lastHeadingDiffDegrees = 0.0;
        boolean measurementAccepted = false;
        String rejectionReason = "No tags";
        double avgTagDistance = 0.0;

        // Read the target validity from the Limelight
        m_hasTarget = LimelightHelpers.getTV(m_limelightName);

        // Always update hub distances (odometry-based and direct tag-based)
        updateHubDistance();
        updateDirectDistanceToHub();

        if (!m_hasTarget) {
            m_tagCount = 0;
            publishTelemetry(lastPoseDiffMeters, lastHeadingDiffDegrees,
                             measurementAccepted, rejectionReason, m_tagCount, avgTagDistance);
            return;
        }

        // Supply the robot's gyro heading to Limelight for MegaTag2 orientation
        LimelightHelpers.SetRobotOrientation(
            m_limelightName,
            m_driveSubsystem.getHeading().getDegrees(),
            0, 0, 0, 0, 0);

        // Retrieve the MegaTag2 pose estimate (alliance-aware)
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        LimelightHelpers.PoseEstimate poseEstimate =
            (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
                ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(m_limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);

        if (poseEstimate == null || poseEstimate.tagCount < VisionConstants.kMinTagCount) {
            m_tagCount = poseEstimate != null ? poseEstimate.tagCount : 0;
            publishTelemetry(lastPoseDiffMeters, lastHeadingDiffDegrees,
                             measurementAccepted, rejectionReason, m_tagCount, avgTagDistance);
            return;
        }

        m_tagCount = poseEstimate.tagCount;
        Pose2d visionPose = poseEstimate.pose;

        // Compute average tag distance
        if (poseEstimate.rawFiducials != null && poseEstimate.rawFiducials.length > 0) {
            double totalDist = 0.0;
            for (LimelightHelpers.RawFiducial fiducial : poseEstimate.rawFiducials) {
                totalDist += fiducial.distToRobot;
            }
            avgTagDistance = totalDist / poseEstimate.rawFiducials.length;
        }

        // Validate the measurement against current odometry to reject outliers
        Pose2d currentPose = m_driveSubsystem.getPose();
        lastPoseDiffMeters = currentPose.getTranslation()
            .getDistance(visionPose.getTranslation());

        if (lastPoseDiffMeters > VisionConstants.kMaxAcceptableDistanceMeters) {
            rejectionReason = "Distance too far";
            publishTelemetry(lastPoseDiffMeters, lastHeadingDiffDegrees,
                             measurementAccepted, rejectionReason, m_tagCount, avgTagDistance);
            return;
        }

        lastHeadingDiffDegrees = Math.abs(
            MathUtil.inputModulus(
                currentPose.getRotation().getDegrees() - visionPose.getRotation().getDegrees(),
                -180, 180));
        if (lastHeadingDiffDegrees > VisionConstants.kMaxAcceptableRotationDegrees) {
            rejectionReason = "Heading mismatch";
            publishTelemetry(lastPoseDiffMeters, lastHeadingDiffDegrees,
                             measurementAccepted, rejectionReason, m_tagCount, avgTagDistance);
            return;
        }

        // Scale standard deviations based on how many tags we see
        double stdDevX;
        double stdDevY;
        double stdDevTheta;

        if (poseEstimate.tagCount >= 2) {
            stdDevX = VisionConstants.kMultiTagStdDevX;
            stdDevY = VisionConstants.kMultiTagStdDevY;
            stdDevTheta = VisionConstants.kMultiTagStdDevTheta;
        } else {
            stdDevX = VisionConstants.kSingleTagStdDevX;
            stdDevY = VisionConstants.kSingleTagStdDevY;
            stdDevTheta = VisionConstants.kSingleTagStdDevTheta;
        }

        m_driveSubsystem.addVisionMeasurement(
            visionPose,
            poseEstimate.timestampSeconds,
            stdDevX, stdDevY, stdDevTheta);

        measurementAccepted = true;
        rejectionReason = "Accepted";
        publishTelemetry(lastPoseDiffMeters, lastHeadingDiffDegrees,
                         measurementAccepted, rejectionReason, m_tagCount, avgTagDistance);
    }

    private void updateHubDistance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            m_distanceToHubMeters = -1.0;
            return;
        }
        double hubX = (alliance.get() == DriverStation.Alliance.Red)
            ? OrbitalConstants.kHubRedX
            : OrbitalConstants.kHubBlueX;
        Translation2d hubTranslation = new Translation2d(hubX, OrbitalConstants.kHubY);
        m_distanceToHubMeters = m_driveSubsystem.getPose()
            .getTranslation()
            .getDistance(hubTranslation);
        SmartDashboard.putNumber("Shooter/Distance To Hub", m_distanceToHubMeters);
    }

    /**
     * Updates {@code m_directDistanceToHubMeters} by scanning the raw fiducials
     * from the latest Limelight frame and averaging the distances of any hub
     * AprilTags that are currently visible.
     *
     * <p>Uses only tags belonging to the current alliance's hub cluster
     * (Blue: IDs 18–21, 24–27 | Red: IDs 2–5, 8–11).
     * Falls back to -1.0 if no hub tags are visible or alliance is unknown.
     */
    private void updateDirectDistanceToHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            m_directDistanceToHubMeters = -1.0;
            SmartDashboard.putNumber("Shooter/Direct Distance To Hub", -1.0);
            return;
        }

        int[] hubTagIds = (alliance.get() == DriverStation.Alliance.Red)
            ? kRedHubTagIds
            : kBlueHubTagIds;

        LimelightHelpers.RawFiducial[] fiducials =
            LimelightHelpers.getRawFiducials(m_limelightName);

        if (fiducials == null || fiducials.length == 0) {
            m_directDistanceToHubMeters = -1.0;
            SmartDashboard.putNumber("Shooter/Direct Distance To Hub", -1.0);
            return;
        }

        double totalDist = 0.0;
        int hubTagCount = 0;
        for (LimelightHelpers.RawFiducial fiducial : fiducials) {
            for (int hubId : hubTagIds) {
                if (fiducial.id == hubId) {
                    totalDist += fiducial.distToRobot;
                    hubTagCount++;
                    break;
                }
            }
        }

        if (hubTagCount == 0) {
            m_directDistanceToHubMeters = -1.0;
        } else {
            m_directDistanceToHubMeters = totalDist / hubTagCount;
        }

        SmartDashboard.putNumber("Shooter/Direct Distance To Hub", m_directDistanceToHubMeters);
        SmartDashboard.putNumber("Shooter/Direct Distance Hub Tag Count", hubTagCount);
    }

    private void publishTelemetry(
            double poseDiffMeters,
            double headingDiffDegrees,
            boolean accepted,
            String rejectionReason,
            int tagCount,
            double avgTagDistance) {
        SmartDashboard.putBoolean("Vision/HasTarget", m_hasTarget);
        SmartDashboard.putNumber("Vision/TagCount", tagCount);

        if (Preferences.getBoolean(kVerboseTelemetryKey, false)) {
            SmartDashboard.putNumber("Vision/LastPoseDiffMeters", poseDiffMeters);
            SmartDashboard.putNumber("Vision/LastHeadingDiffDegrees", headingDiffDegrees);
            SmartDashboard.putBoolean("Vision/MeasurementAccepted", accepted);
            SmartDashboard.putString("Vision/RejectionReason", rejectionReason);
            SmartDashboard.putNumber("Vision/AvgTagDistance", avgTagDistance);
        }
    }
}
