package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

class ShootCommandTest {

    private ShooterSubsystem m_shooter;
    private VisionSubsystem  m_vision;
    private ShootCommand m_command;

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        CommandScheduler.getInstance().cancelAll();
        m_shooter = mock(ShooterSubsystem.class);
        m_vision  = mock(VisionSubsystem.class);
        // Default: no vision target — command uses fallback RPMs
        when(m_vision.getDistanceToHub()).thenReturn(-1.0);
        m_command = new ShootCommand(m_shooter, m_vision);
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
    void initializeCallsRunShooterAtRPMAndRunPreShooterAtRPM() {
        m_command.initialize();

        verify(m_shooter).runShooterAtRPM(anyDouble());
        verify(m_shooter).runPreShooterAtRPM(anyDouble());
    }

    @Test
    void executeDoesNotFeedBeforeShooterAtSpeed() {
        when(m_shooter.isShooterAtSpeed(anyDouble())).thenReturn(false);

        m_command.initialize();
        m_command.execute();

        verify(m_shooter, never()).runAgitator();
    }

    @Test
    void executeStartsFeedingOnceShooterAtSpeed() {
        when(m_shooter.isShooterAtSpeed(anyDouble())).thenReturn(true);

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

    @Test
    void initializeUsesInterpolatedRPMWhenDistanceIsValid() {
        when(m_vision.getDistanceToHub()).thenReturn(2.5);

        m_command.initialize();

        // With distance 2.5m the interpolated shooter RPM should be 4400.0
        verify(m_shooter).runShooterAtRPM(4400.0);
    }
}
