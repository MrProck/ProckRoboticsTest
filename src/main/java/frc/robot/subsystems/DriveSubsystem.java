package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule m_frontLeft = new SwerveModule(
        SwerveConstants.kFLDriveMotorID, SwerveConstants.kFLSteerMotorID,
        SwerveConstants.kFLCANcoderID, SwerveConstants.kFLCANcoderOffset,
        SwerveConstants.kFLDriveInverted, SwerveConstants.kFLSteerInverted);
    private final SwerveModule m_frontRight = new SwerveModule(
        SwerveConstants.kFRDriveMotorID, SwerveConstants.kFRSteerMotorID,
        SwerveConstants.kFRCANcoderID, SwerveConstants.kFRCANcoderOffset,
        SwerveConstants.kFRDriveInverted, SwerveConstants.kFRSteerInverted);
    private final SwerveModule m_backLeft = new SwerveModule(
        SwerveConstants.kBLDriveMotorID, SwerveConstants.kBLSteerMotorID,
        SwerveConstants.kBLCANcoderID, SwerveConstants.kBLCANcoderOffset,
        SwerveConstants.kBLDriveInverted, SwerveConstants.kBLSteerInverted);
    private final SwerveModule m_backRight = new SwerveModule(
        SwerveConstants.kBRDriveMotorID, SwerveConstants.kBRSteerMotorID,
        SwerveConstants.kBRCANcoderID, SwerveConstants.kBRCANcoderOffset,
        SwerveConstants.kBRDriveInverted, SwerveConstants.kBRSteerInverted);

    private final Pigeon2 m_pigeon = new Pigeon2(SwerveConstants.kPigeonID, SwerveConstants.kCANivoreName);

    private final SwerveDrivePoseEstimator m_poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveConstants.kSwerveKinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d());

    private final Field2d m_field = new Field2d();

    public DriveSubsystem() {
        SmartDashboard.putData("Field", m_field);
        zeroHeading();
    }

    public void drive(double xSpeedMPS, double ySpeedMPS, double rotRadPerSec, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMPS, ySpeedMPS, rotRadPerSec, getHeading())
            : new ChassisSpeeds(xSpeedMPS, ySpeedMPS, rotRadPerSec);

        SwerveModuleState[] moduleStates =
            SwerveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxDriveSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(moduleStates[0]);
        m_frontRight.setDesiredState(moduleStates[1]);
        m_backLeft.setDesiredState(moduleStates[2]);
        m_backRight.setDesiredState(moduleStates[3]);
    }

    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxDriveSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-m_pigeon.getYaw().getValueAsDouble());
    }

    public void zeroHeading() {
        m_pigeon.reset();
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(getHeading(), getModulePositions());
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("Gyro Heading (deg)", getHeading().getDegrees());
        SmartDashboard.putNumber("Robot X (m)", getPose().getX());
        SmartDashboard.putNumber("Robot Y (m)", getPose().getY());
    }
}