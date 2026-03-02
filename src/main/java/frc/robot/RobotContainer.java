package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();

        // Default drive command — field-centric swerve
        // Left stick Y = forward/back, Left stick X = strafe, Right stick X = rotate
        m_driveSubsystem.setDefaultCommand(
            new TeleopDriveCommand(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX()
            )
        );
    }

    private void configureBindings() {
        // Start button — zero gyro heading (reset field-centric orientation)
        m_driverController.start().onTrue(
            new InstantCommand(m_driveSubsystem::zeroHeading, m_driveSubsystem)
        );
    }

    public Command getAutonomousCommand() {
        // Placeholder — wire PathPlanner or auto command here
        return new InstantCommand();
    }
}