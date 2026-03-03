package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonomousDriveCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final DriveSubsystem  m_driveSubsystem  = new DriveSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_driveSubsystem);

    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OIConstants.kOperatorControllerPort);

    public RobotContainer() {
        configureBindings();

        // Default drive command — field-centric swerve
        m_driveSubsystem.setDefaultCommand(
            new TeleopDriveCommand(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX()
            )
        );

        // Default intake command — left stick Y controls extension arm
        // Push forward = extend, pull back = retract
        m_intakeSubsystem.setDefaultCommand(
            new RunCommand(() -> {
                double stickY = MathUtil.applyDeadband(-m_operatorController.getLeftY(), OIConstants.kDriveDeadband);
                if (stickY > 0) {
                    m_intakeSubsystem.extend();
                } else if (stickY < 0) {
                    m_intakeSubsystem.retract();
                }
            }, m_intakeSubsystem)
        );
    }

    private void configureBindings() {
        // ---- Driver Controller ----

        // Start — zero gyro heading
        m_driverController.start().onTrue(
            new InstantCommand(m_driveSubsystem::zeroHeading, m_driveSubsystem)
        );

        // ---- Operator Controller ----

        // Right Trigger (held) — run intake roller forward
        // Won't run if intake is locked out (enforced inside IntakeSubsystem)
        m_operatorController.rightTrigger(IntakeConstants.kTriggerThreshold)
            .whileTrue(new RunCommand(m_intakeSubsystem::runIntake, m_intakeSubsystem))
            .onFalse(new InstantCommand(m_intakeSubsystem::stopRoller, m_intakeSubsystem));

        // Right Bumper (held) — eject (roller reverse, always allowed)
        m_operatorController.rightBumper()
            .whileTrue(new RunCommand(m_intakeSubsystem::runEject, m_intakeSubsystem))
            .onFalse(new InstantCommand(m_intakeSubsystem::stopRoller, m_intakeSubsystem));

        // Back button — clear intake lockout after ejecting wrong game piece
        m_operatorController.back().onTrue(
            new InstantCommand(m_intakeSubsystem::clearLockout, m_intakeSubsystem)
        );
    }

    public Command getAutonomousCommand() {
        return new AutonomousDriveCommand(m_driveSubsystem);
    }
}
