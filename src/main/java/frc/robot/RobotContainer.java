package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterTableConstants;
import frc.robot.commands.AutonomousDriveCommand;
import frc.robot.commands.IntakeExtensionCommand;
import frc.robot.commands.HerdDriveCommand;
import frc.robot.commands.OrbitalDriveCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final DriveSubsystem   m_driveSubsystem   = new DriveSubsystem();
    private final IntakeSubsystem  m_intakeSubsystem  = new IntakeSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final VisionSubsystem  m_visionSubsystem  = new VisionSubsystem(m_driveSubsystem);

    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OIConstants.kOperatorControllerPort);

    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    // Toggled by the driver Back button — true = robot-centric, false = field-centric (default)
    private boolean m_robotCentric = false;

    public RobotContainer() {
        // Register named commands for PathPlanner BEFORE configureAutoChooser().
        // These names must exactly match the command names used in PathPlanner auto files.
        NamedCommands.registerCommand("Shoot",
            new AutoShootCommand(m_driveSubsystem, m_shooterSubsystem, m_visionSubsystem)
                .withTimeout(ShooterConstants.kShooterSpinUpTimeoutSeconds + 2.0));

        NamedCommands.registerCommand("IntakeIn",
            new RunCommand(() -> {
                m_intakeSubsystem.runIntake();
                m_shooterSubsystem.runAgitatorAtRPM(ShooterConstants.kAgitatorForwardRPM);
            }, m_intakeSubsystem)  // no m_shooterSubsystem requirement — avoids conflicting with Shoot command
                .withTimeout(2.0));

        NamedCommands.registerCommand("IntakeStop",
            new InstantCommand(() -> {
                m_intakeSubsystem.stopRoller();
                m_shooterSubsystem.stopAgitator();
            }, m_intakeSubsystem));  // no m_shooterSubsystem requirement

        configureBindings();
        configureAutoChooser();

        // Default drive command — field-centric swerve
        // Press Back to toggle robot-centric driving on/off
        m_driveSubsystem.setDefaultCommand(
            new TeleopDriveCommand(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX(),
                () -> 1.0,   // always full speed — trigger no longer used for acceleration
                () -> m_robotCentric
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
            m_autoChooser.addOption("Depot Shoot N Scoot", AutoBuilder.buildAuto("DepotShootNScoot"));
        } catch (Exception e) {
            System.err.println("[RobotContainer] Could not load 'DepotShootNScoot' auto: " + e.getMessage());
        }

        try {
            m_autoChooser.addOption("Human Shoot N Scoot", AutoBuilder.buildAuto("HumanShootNScoot"));
        } catch (Exception e) {
            System.err.println("[RobotContainer] Could not load 'HumanShootNScoot' auto: " + e.getMessage());
        }

        try {
            m_autoChooser.addOption("Tower Shoot N Scoot", AutoBuilder.buildAuto("TowerShootNScoot"));
        } catch (Exception e) {
            System.err.println("[RobotContainer] Could not load 'TowerShootNScoot' auto: " + e.getMessage());
        }

        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    private void configureBindings() {
        // ---- Driver Controller ----

        // Start — zero gyro heading
        m_driverController.start().onTrue(
            new InstantCommand(m_driveSubsystem::zeroHeading, m_driveSubsystem)
        );

        // Back — toggle robot-centric driving on/off
        m_driverController.back().onTrue(
            new InstantCommand(() -> m_robotCentric = !m_robotCentric)
        );

        // Left Bumper (held) — orbital mode: lock heading toward 2026 REBUILT field hub
        m_driverController.leftBumper()
            .whileTrue(new OrbitalDriveCommand(
                m_driveSubsystem,
                m_visionSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () ->  1.0,   // always full speed — trigger no longer used for acceleration
                () -> -m_driverController.getRightY()
            ));

        // Left Trigger (held) — driver intake: run roller + agitator
        m_driverController.leftTrigger(OIConstants.kTriggerThreshold)
            .whileTrue(createIntakeRunCommand())
            .onFalse(createIntakeStopCommand());

        // X (held) — pre-spin shooter flywheels only (no feeding)
        // Uses fallback RPM for shooter so pre-spin matches what ShootCommand will use
        m_driverController.x()
            .whileTrue(Commands.startEnd(
                () -> {
                    m_shooterSubsystem.runPreShooterAtRPM(ShooterConstants.kPreShooterForwardRPM);
                    m_shooterSubsystem.runShooterAtRPM(ShooterTableConstants.kFallbackShooterRPM);
                },
                () -> {
                    m_shooterSubsystem.stopPreShooter();
                    m_shooterSubsystem.stopShooter();
                },
                m_shooterSubsystem
            ));

        // A (held) — full shoot sequence (spin up + feed when at speed)
        m_driverController.a()
            .whileTrue(new ShootCommand(m_shooterSubsystem, m_visionSubsystem));

        // B (held) — wide herd mode: wide side faces direction of travel
        m_driverController.b()
            .whileTrue(new HerdDriveCommand(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () ->  m_driverController.getRightTriggerAxis(),
                HerdDriveCommand.Mode.WIDE
            ));

        // Y (held) — narrow herd mode: corner faces direction of travel
        m_driverController.y()
            .whileTrue(new HerdDriveCommand(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () ->  m_driverController.getRightTriggerAxis(),
                HerdDriveCommand.Mode.NARROW
            ));

        // Right Bumper (held) — reverse all shooter stages to clear jams
        m_driverController.rightBumper()
            .whileTrue(new RunCommand(m_shooterSubsystem::reverseAll, m_shooterSubsystem))
            .onFalse(new InstantCommand(m_shooterSubsystem::stopAll, m_shooterSubsystem));

        // ---- Operator Controller ----

        // Right Trigger (held) — run intake roller forward + agitator
        // Uses forward RPM (higher) so it doesn't conflict with shooter.
        // Won't run if intake is locked out (enforced inside IntakeSubsystem)
        m_operatorController.rightTrigger(OIConstants.kTriggerThreshold)
            .whileTrue(createIntakeRunCommand())
            .onFalse(createIntakeStopCommand());

        // Right Bumper (held) — eject (roller reverse, always allowed)
        m_operatorController.rightBumper()
            .whileTrue(new RunCommand(m_intakeSubsystem::runEject, m_intakeSubsystem))
            .onFalse(new InstantCommand(m_intakeSubsystem::stopRoller, m_intakeSubsystem));

        // Back button — clear intake lockout after ejecting wrong game piece
        m_operatorController.back().onTrue(
            new InstantCommand(m_intakeSubsystem::clearLockout, m_intakeSubsystem)
        );

        // Left Trigger (>10%) — spin up shooter flywheels only (pre-spin, no feeding)
        // Uses fallback RPM for shooter so pre-spin matches what ShootCommand will use
        m_operatorController.leftTrigger(0.1)
            .and(m_operatorController.leftTrigger(0.9).negate())
            .whileTrue(Commands.startEnd(
                () -> {
                    m_shooterSubsystem.runPreShooterAtRPM(ShooterConstants.kPreShooterForwardRPM);
                    m_shooterSubsystem.runShooterAtRPM(ShooterTableConstants.kFallbackShooterRPM);
                },
                () -> {
                    m_shooterSubsystem.stopPreShooter();
                    m_shooterSubsystem.stopShooter();
                },
                m_shooterSubsystem
            ));

        // Left Trigger (>90%) — full shoot sequence (spin up + feed when at speed)
        m_operatorController.leftTrigger(0.9)
            .whileTrue(new ShootCommand(m_shooterSubsystem, m_visionSubsystem));

        // Left Bumper (held) — manual reverse all shooter stages to clear jams
        m_operatorController.leftBumper()
            .whileTrue(new RunCommand(() -> {
                m_shooterSubsystem.reverseAll();
            }, m_shooterSubsystem))
            .onFalse(new InstantCommand(m_shooterSubsystem::stopAll, m_shooterSubsystem));

        // A (held) — manual fixed-RPM shoot (bypasses vision/distance interpolation)
        // Spins up to "Shooter/Target RPM" (set by D-pad), then feeds when at speed.
        m_operatorController.a()
            .whileTrue(createManualShootCommand());

        // D-pad Up — increase default shooter RPM by step (no subsystem requirement so it
        // doesn't interrupt the running manual shoot command)
        m_operatorController.povUp().onTrue(
            new InstantCommand(
                () -> m_shooterSubsystem.adjustDefaultRPM(ShooterConstants.kManualShootRPMStep)));

        // D-pad Down — decrease default shooter RPM by step (no subsystem requirement so it
        // doesn't interrupt the running manual shoot command)
        m_operatorController.povDown().onTrue(
            new InstantCommand(
                () -> m_shooterSubsystem.adjustDefaultRPM(-ShooterConstants.kManualShootRPMStep)));
    }

    /** Runs the intake roller and agitator together (used by both driver and operator triggers). */
    private Command createIntakeRunCommand() {
        return new RunCommand(() -> {
            m_intakeSubsystem.runIntake();
            // Agitator runs in reverse during intake to feed game piece toward the intake roller.
            // During shoot, ShootCommand calls runAgitator() (forward) and owns m_shooterSubsystem,
            // so it takes over — this reverse call is simply not running when shoot is active.
            m_shooterSubsystem.runAgitatorAtRPM(-ShooterConstants.kAgitatorReverseRPM);
            // Run flywheels in reverse during intake to pull the game piece toward the shooter.
            // ShootCommand has m_shooterSubsystem as a requirement so it will take over the
            // motor output the moment shoot is pressed — this call is simply ignored/overridden.
            m_shooterSubsystem.runPreShooterAtRPM(-ShooterConstants.kPreShooterForwardRPM);
            m_shooterSubsystem.runShooterAtRPM(-ShooterConstants.kShooterForwardRPM);
        }, m_intakeSubsystem);  // intentionally no m_shooterSubsystem requirement — ShootCommand must not be interrupted by intake
    }

    /** Stops the intake roller and agitator (used by both driver and operator triggers). */
    private Command createIntakeStopCommand() {
        return new InstantCommand(() -> {
            m_intakeSubsystem.stopRoller();
            m_shooterSubsystem.stopAgitator();
            m_shooterSubsystem.stopPreShooter();
            m_shooterSubsystem.stopShooter();
        }, m_intakeSubsystem);  // intentionally no m_shooterSubsystem requirement
    }

    /**
     * Creates a command that shoots at the current Preferences "Shooter/Target RPM",
     * bypassing distance-based interpolation. Follows the same kicker/agitator
     * sequencing as AutoShootCommand but without heading PID — the driver aims manually:
     *
     * <ol>
     *   <li>Kicker held in slow reverse during spin-up to prevent pre-loading.
     *   <li>Once at speed (or timeout): kicker runs forward.
     *   <li>Once kicker is up to feeding speed: agitator engages.
     * </ol>
     *
     * Runs until cancelled (button released).
     */
    private Command createManualShootCommand() {
        return new Command() {
            private final edu.wpi.first.wpilibj.Timer m_timer = new edu.wpi.first.wpilibj.Timer();
            private double m_rpm;

            {
                addRequirements(m_shooterSubsystem);
            }

            @Override
            public void initialize() {
                m_timer.restart();
                // Start spinning up; hold kicker in slow reverse to prevent pre-loading
                m_shooterSubsystem.runShooterAtRPM(
                    edu.wpi.first.wpilibj.Preferences.getDouble(
                        "Shooter/Target RPM", ShooterConstants.kShooterForwardRPM));
                m_shooterSubsystem.runPreShooterAtRPM(ShooterConstants.kPreShooterForwardRPM);
                m_shooterSubsystem.runKickerSlowReverse();
            }

            @Override
            public void execute() {
                // Re-read Preferences every cycle so D-pad adjustments apply immediately
                m_rpm = edu.wpi.first.wpilibj.Preferences.getDouble(
                    "Shooter/Target RPM", ShooterConstants.kShooterForwardRPM);
                SmartDashboard.putNumber("Shooter/Manual Shot RPM", m_rpm);

                // --- Spin-up ---
                m_shooterSubsystem.runShooterAtRPM(m_rpm);
                m_shooterSubsystem.runPreShooterAtRPM(ShooterConstants.kPreShooterForwardRPM);

                boolean atSpeed  = m_shooterSubsystem.isShooterAtSpeed(m_rpm);
                boolean timedOut = m_timer.hasElapsed(ShooterConstants.kShooterSpinUpTimeoutSeconds);

                SmartDashboard.putBoolean("Shooter/FeedGate AtSpeed",  atSpeed);
                SmartDashboard.putBoolean("Shooter/FeedGate TimedOut", timedOut);

                // Feed once at speed (or timed out); wait for kicker before agitator
                if (atSpeed || timedOut) {
                    m_shooterSubsystem.runKicker();
                    if (m_shooterSubsystem.isKickerFeeding()) {
                        m_shooterSubsystem.runAgitator();
                    }
                } else {
                    m_shooterSubsystem.runKickerSlowReverse();
                }
            }

            @Override
            public void end(boolean interrupted) {
                m_shooterSubsystem.stopAll();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    /** Returns the drive subsystem for simulation use. */
    public DriveSubsystem getDriveSubsystem() {
        return m_driveSubsystem;
    }

    /** Returns the intake subsystem for use outside this class (e.g., Robot.java). */
    public IntakeSubsystem getIntakeSubsystem() {
        return m_intakeSubsystem;
    }
}
