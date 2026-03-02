package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final CANSparkMax m_leftMotor1 =
        new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_leftMotor2 =
        new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor1 =
        new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor2 =
        new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

    @SuppressWarnings("deprecation")
    private final MotorControllerGroup m_leftMotors =
        new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    @SuppressWarnings("deprecation")
    private final MotorControllerGroup m_rightMotors =
        new MotorControllerGroup(m_rightMotor1, m_rightMotor2);

    private final DifferentialDrive m_drive =
        new DifferentialDrive(m_leftMotors, m_rightMotors);

    public DriveSubsystem() {
        m_leftMotor1.restoreFactoryDefaults();
        m_leftMotor2.restoreFactoryDefaults();
        m_rightMotor1.restoreFactoryDefaults();
        m_rightMotor2.restoreFactoryDefaults();

        m_rightMotors.setInverted(true);
        m_drive.setDeadband(DriveConstants.kDeadband);
    }

    /**
     * Arcade drive for differential drive platform.
     * @param fwd  forward movement [-1.0, 1.0]
     * @param rot  rotation [-1.0, 1.0]
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Tank drive for differential drive platform.
     * @param leftSpeed  left side speed [-1.0, 1.0]
     * @param rightSpeed right side speed [-1.0, 1.0]
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    /** Stops all drive motors. */
    public void stopDrive() {
        m_drive.stopMotor();
    }

    @Override
    public void periodic() {
        // Add SmartDashboard telemetry here
    }
}
