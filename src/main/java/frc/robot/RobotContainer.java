package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonomousDriveCommand;
import frc.robot.commands.IntakeExtensionCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final DriveSubsystem   m_driveSubsystem   = new DriveSubsystem();
    private final IntakeSubsystem  m_intakeSubsystem  = new IntakeSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    // VisionSubsystem runs via its periodic() method — no direct references needed
    @SuppressWarnings("unused")
    private final VisionSubsystem  m_visionSubsystem  = new VisionSubsystem(m_driveSubsystem);

    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OIConstants.kOperatorControllerPort);

    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutoChooser();

        // Default drive command — field-centric swerve
        m_driveSubsystem.setDefaultCommand(
            new TeleopDriveCommand(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX(),
                () -> m_driverController.getRightTriggerAxis()
            )
        );

        // Default intake command — left stick Y controls extension arm
        // Push forward = extend, pull back = retract
        m_intakeSubsystem.setDefaultCommand(
            new IntakeExtensionCommand(
                m_intakeSubsystem,
                () -> -m_operatorController.getLeftY()
            )
        );
    }

    private void configureAutoChooser() {
        // "Do Nothing" — default option that simply stops the drive
        m_autoChooser.setDefaultOption("Do Nothing",
            Commands.runOnce(m_driveSubsystem::stopModules, m_driveSubsystem));

        // Timed drive-forward fallback
        m_autoChooser.addOption("Drive Forward", new AutonomousDriveCommand(m_driveSubsystem));

        // PathPlanner-based autos (path files are in deploy/pathplanner/autos/)
        try {
            m_autoChooser.addOption("Just Leave", AutoBuilder.buildAuto("Just Leave"));
        } catch (Exception e) {
            System.err.println("[RobotContainer] Could not load 'Just Leave' auto: " + e.getMessage());
        }

        try {
            m_autoChooser.addOption("Score And Leave", AutoBuilder.buildAuto("Score And Leave"));
        } catch (Exception e) {
            System.err.println("[RobotContainer] Could not load 'Score And Leave' auto: " + e.getMessage());
        }

        SmartDashboard.putData("Auto Chooser", m_autoChooser);
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
        m_operatorController.rightTrigger(OIConstants.kTriggerThreshold)
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

        // Left Trigger (held) — run full shoot sequence (spin up, then feed when at speed)
        m_operatorController.leftTrigger(OIConstants.kTriggerThreshold)
            .whileTrue(new ShootCommand(m_shooterSubsystem));

        // Left Bumper (held) — manual reverse all shooter stages to clear jams
        m_operatorController.leftBumper()
            .whileTrue(new RunCommand(() -> {
                m_shooterSubsystem.reverseAll();
            }, m_shooterSubsystem))
            .onFalse(new InstantCommand(m_shooterSubsystem::stopAll, m_shooterSubsystem));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    /** Returns the intake subsystem for use outside this class (e.g., Robot.java). */
    public IntakeSubsystem getIntakeSubsystem() {
        return m_intakeSubsystem;
    }
}
