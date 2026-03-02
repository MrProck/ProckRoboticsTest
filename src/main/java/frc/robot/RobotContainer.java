package frc.robot;

import frc.robot.commands.AutonomousDriveCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

    private final CommandXboxController m_driverController =
        new CommandXboxController(Constants.OIConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();

        m_driveSubsystem.setDefaultCommand(
            new TeleopDriveCommand(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getRightX()
            )
        );
    }

    private void configureBindings() {
        // Add button bindings here
        // Example: m_driverController.a().whileTrue(new ExampleCommand(m_driveSubsystem));
    }

    public Command getAutonomousCommand() {
        return new AutonomousDriveCommand(m_driveSubsystem);
    }
}
