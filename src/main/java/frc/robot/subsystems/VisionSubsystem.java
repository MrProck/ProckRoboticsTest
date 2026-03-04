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
        // Read the target validity from the Limelight
        m_hasTarget = LimelightHelpers.getTV(m_limelightName);

        if (!m_hasTarget) {
            m_tagCount = 0;
            publishTelemetry();
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
            publishTelemetry();
            return;
        }

        m_tagCount = poseEstimate.tagCount;
        Pose2d visionPose = poseEstimate.pose;

        // Validate the measurement against current odometry to reject outliers
        Pose2d currentPose = m_driveSubsystem.getPose();
        double poseDifference = currentPose.getTranslation()
            .getDistance(visionPose.getTranslation());

        if (poseDifference > VisionConstants.kMaxAcceptableDistanceMeters) {
            publishTelemetry();
            return;
        }

        double headingDifference = Math.abs(
            MathUtil.inputModulus(
                currentPose.getRotation().getDegrees() - visionPose.getRotation().getDegrees(),
                -180, 180));
        if (headingDifference > VisionConstants.kMaxAcceptableRotationDegrees) {
            publishTelemetry();
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

        publishTelemetry();
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("Vision/HasTarget", m_hasTarget);
        SmartDashboard.putNumber("Vision/TagCount", m_tagCount);
    }
}
