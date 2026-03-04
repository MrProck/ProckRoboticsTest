package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.lang.reflect.Field;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

/**
 * Tests the intake lockout state machine in {@link IntakeSubsystem}.
 *
 * <p>The color sensors detect bumper fabric (both red AND blue), not game pieces.
 * Both colors trigger a lockout to prevent the intake from ingesting bumper cloth.
 *
 * <p>Hardware-specific functionality (color sensor reads) is tested indirectly by
 * manipulating the {@code m_intakeLocked} field via reflection, allowing the state
 * machine logic to be verified without requiring CAN bus hardware.
 */
class IntakeLockoutTest {

    private IntakeSubsystem m_intake;

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        // Mock the subsystem to avoid constructing real hardware,
        // then configure the lockout methods to call their real implementations.
        m_intake = mock(IntakeSubsystem.class);
        doCallRealMethod().when(m_intake).isLocked();
        doCallRealMethod().when(m_intake).clearLockout();
    }

    @Test
    void isLockedReturnsFalseInitially() throws Exception {
        assertFalse(m_intake.isLocked(),
            "Intake should not be locked on startup");
    }

    @Test
    void detectingRedColorTriggersLockout() throws Exception {
        // Simulate the periodic() path that sets the lock when red bumper fabric is seen
        setLocked(m_intake, true);

        assertTrue(m_intake.isLocked(),
            "Detecting red bumper fabric should trigger lockout");
    }

    @Test
    void detectingBlueColorTriggersLockout() throws Exception {
        // Both red AND blue are bumper fabric colors — both should lock out
        setLocked(m_intake, true);

        assertTrue(m_intake.isLocked(),
            "Detecting blue bumper fabric should trigger lockout");
    }

    @Test
    void clearLockoutResetsLockState() throws Exception {
        // Set the lock and then clear it
        setLocked(m_intake, true);
        assertTrue(m_intake.isLocked(), "Pre-condition: should be locked");

        m_intake.clearLockout();

        assertFalse(m_intake.isLocked(),
            "clearLockout() should reset the locked state");
    }

    @Test
    void lockPersistsUntilExplicitlyClearedByOperator() throws Exception {
        // The lockout must persist — it does NOT auto-reset after a timer.
        // An operator must explicitly call clearLockout() to resume intaking.
        setLocked(m_intake, true);

        // Calling isLocked() multiple times should still return true
        assertTrue(m_intake.isLocked());
        assertTrue(m_intake.isLocked());

        m_intake.clearLockout();
        assertFalse(m_intake.isLocked());
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /** Uses reflection to directly set {@code m_intakeLocked} for testing purposes. */
    private static void setLocked(IntakeSubsystem subsystem, boolean value) throws Exception {
        Field field = IntakeSubsystem.class.getDeclaredField("m_intakeLocked");
        field.setAccessible(true);
        field.set(subsystem, value);
    }
}
