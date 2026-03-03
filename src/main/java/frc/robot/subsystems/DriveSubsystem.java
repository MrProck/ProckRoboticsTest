package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PathPlannerConstants;
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
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        RobotConfig robotConfig;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Fall back to a programmatic config derived from known robot constants
            ModuleConfig moduleConfig = new ModuleConfig(
                SwerveConstants.kWheelDiameterMeters / 2.0,
                SwerveConstants.kMaxDriveSpeedMetersPerSecond,
                1.2,
                DCMotor.getFalcon500(1).withReduction(SwerveConstants.kDriveGearRatio),
                SwerveConstants.kDriveCurrentLimitAmps,
                1);
            robotConfig = new RobotConfig(
                60.0,
                6.0,
                moduleConfig,
                new Translation2d( SwerveConstants.kWheelBaseMeters / 2.0,  SwerveConstants.kTrackWidthMeters / 2.0),
                new Translation2d( SwerveConstants.kWheelBaseMeters / 2.0, -SwerveConstants.kTrackWidthMeters / 2.0),
                new Translation2d(-SwerveConstants.kWheelBaseMeters / 2.0,  SwerveConstants.kTrackWidthMeters / 2.0),
                new Translation2d(-SwerveConstants.kWheelBaseMeters / 2.0, -SwerveConstants.kTrackWidthMeters / 2.0));
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> {
                SwerveModuleState[] states =
                    SwerveConstants.kSwerveKinematics.toSwerveModuleStates(speeds);
                setModuleStates(states);
            },
            new PPHolonomicDriveController(
                new PIDConstants(PathPlannerConstants.kTranslationP,
                                 PathPlannerConstants.kTranslationI,
                                 PathPlannerConstants.kTranslationD),
                new PIDConstants(PathPlannerConstants.kRotationP,
                                 PathPlannerConstants.kRotationI,
                                 PathPlannerConstants.kRotationD)),
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this);
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

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose       The pose measured by the vision system.
     * @param timestampSeconds The timestamp of the vision measurement (FPGA time).
     * @param stdDevsX         Standard deviation of the x measurement (meters).
     * @param stdDevsY         Standard deviation of the y measurement (meters).
     * @param stdDevsTheta     Standard deviation of the heading measurement (radians).
     */
    public void addVisionMeasurement(
            Pose2d visionPose,
            double timestampSeconds,
            double stdDevsX,
            double stdDevsY,
            double stdDevsTheta) {
        m_poseEstimator.addVisionMeasurement(
            visionPose,
            timestampSeconds,
            VecBuilder.fill(stdDevsX, stdDevsY, stdDevsTheta));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    /** Returns the current robot-relative chassis speeds derived from module states. */
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(getHeading(), getModulePositions());
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("Gyro Heading (deg)", getHeading().getDegrees());
        SmartDashboard.putNumber("Robot X (m)", getPose().getX());
        SmartDashboard.putNumber("Robot Y (m)", getPose().getY());

        SwerveModuleState[] states = getModuleStates();
        SmartDashboard.putNumber("FL Speed (m/s)", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FL Angle (deg)", states[0].angle.getDegrees());
        SmartDashboard.putNumber("FR Speed (m/s)", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("FR Angle (deg)", states[1].angle.getDegrees());
        SmartDashboard.putNumber("BL Speed (m/s)", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BL Angle (deg)", states[2].angle.getDegrees());
        SmartDashboard.putNumber("BR Speed (m/s)", states[3].speedMetersPerSecond);
        SmartDashboard.putNumber("BR Angle (deg)", states[3].angle.getDegrees());

        ChassisSpeeds speeds = SwerveConstants.kSwerveKinematics.toChassisSpeeds(states);
        SmartDashboard.putNumber("ChassisSpeeds vx (m/s)", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeeds vy (m/s)", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeeds omega (rad/s)", speeds.omegaRadiansPerSecond);

        SmartDashboard.putNumberArray("SwerveModuleStates", new double[] {
            states[0].speedMetersPerSecond, states[0].angle.getDegrees(),
            states[1].speedMetersPerSecond, states[1].angle.getDegrees(),
            states[2].speedMetersPerSecond, states[2].angle.getDegrees(),
            states[3].speedMetersPerSecond, states[3].angle.getDegrees()
        });
    }
}
