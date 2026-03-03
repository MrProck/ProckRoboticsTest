package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Default command for the intake subsystem.
 * Uses the operator's left stick Y axis to extend/retract the intake arm.
 * Holds position when the stick is in the deadband.
 */
public class IntakeExtensionCommand extends Command {

    private final IntakeSubsystem m_intakeSubsystem;
    private final DoubleSupplier m_stickInput;

    public IntakeExtensionCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier stickInput) {
        m_intakeSubsystem = intakeSubsystem;
        m_stickInput = stickInput;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        double stickY = MathUtil.applyDeadband(m_stickInput.getAsDouble(), OIConstants.kDriveDeadband);
        if (stickY > 0) {
            m_intakeSubsystem.extend();
        } else if (stickY < 0) {
            m_intakeSubsystem.retract();
        } else {
            m_intakeSubsystem.holdPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
