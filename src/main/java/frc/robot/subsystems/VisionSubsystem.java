package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

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

    private boolean m_hasTarget = false;
    private int m_tagCount = 0;

    /**
     * Creates a new VisionSubsystem.
     *
     * @param driveSubsystem The drive subsystem to send vision poses to.
     */
    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_limelightName = VisionConstants.kLimelightName;

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

        // Retrieve the MegaTag2 pose estimate
        LimelightHelpers.PoseEstimate poseEstimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);

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

    private void publishTelemetry(
            double poseDiffMeters,
            double headingDiffDegrees,
            boolean accepted,
            String rejectionReason,
            int tagCount,
            double avgTagDistance) {
        SmartDashboard.putBoolean("Vision/HasTarget", m_hasTarget);
        SmartDashboard.putNumber("Vision/TagCount", tagCount);
        SmartDashboard.putNumber("Vision/LastPoseDiffMeters", poseDiffMeters);
        SmartDashboard.putNumber("Vision/LastHeadingDiffDegrees", headingDiffDegrees);
        SmartDashboard.putBoolean("Vision/MeasurementAccepted", accepted);
        SmartDashboard.putString("Vision/RejectionReason", rejectionReason);
        SmartDashboard.putNumber("Vision/AvgTagDistance", avgTagDistance);
    }
}
