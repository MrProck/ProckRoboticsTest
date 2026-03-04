package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;

class ShootCommandTest {

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
        // Reset the scheduler state before each test class
        CommandScheduler.getInstance().cancelAll();
    }

    @Test
    void isFinishedReturnsFalse() {
        ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        ShootCommand command = new ShootCommand(shooterSubsystem);
        assertFalse(command.isFinished(), "ShootCommand should run until cancelled");
    }

    @Test
    void constructorAddsShooterSubsystemAsRequirement() {
        ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        ShootCommand command = new ShootCommand(shooterSubsystem);
        assertTrue(
            command.getRequirements().contains(shooterSubsystem),
            "ShootCommand should require the ShooterSubsystem");
    }
}
