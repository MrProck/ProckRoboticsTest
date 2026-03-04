package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;

class ShootCommandTest {

    private ShooterSubsystem m_shooter;
    private ShootCommand m_command;

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        CommandScheduler.getInstance().cancelAll();
        m_shooter = mock(ShooterSubsystem.class);
        m_command = new ShootCommand(m_shooter);
    }

    @Test
    void isFinishedReturnsFalse() {
        assertFalse(m_command.isFinished(), "ShootCommand should run until cancelled");
    }

    @Test
    void constructorAddsShooterSubsystemAsRequirement() {
        assertTrue(
            m_command.getRequirements().contains(m_shooter),
            "ShootCommand should require the ShooterSubsystem");
    }

    @Test
    void initializeCallsRunPreShooterAndRunShooter() {
        m_command.initialize();

        verify(m_shooter).runPreShooter();
        verify(m_shooter).runShooter();
    }

    @Test
    void executeDoesNotFeedBeforeShooterAtSpeed() {
        when(m_shooter.isShooterAtSpeed()).thenReturn(false);

        m_command.initialize();
        m_command.execute();

        verify(m_shooter, never()).runAgitator();
        verify(m_shooter, never()).runKicker();
    }

    @Test
    void executeStartsFeedingOnceShooterAtSpeed() {
        when(m_shooter.isShooterAtSpeed()).thenReturn(true);

        m_command.initialize();
        m_command.execute();

        verify(m_shooter).runAgitator();
        verify(m_shooter).runKicker();
    }

    @Test
    void endCallsStopAll() {
        m_command.end(false);

        verify(m_shooter).stopAll();
    }
}
