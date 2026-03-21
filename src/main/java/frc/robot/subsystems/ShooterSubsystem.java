package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.util.REVUtil;

/**
 * Shooter subsystem with a 4-stage shooting pipeline:
 *  - Agitator    (NEO + SparkMax, CAN 19, 30A)
 *  - Kicker      (NEO Vortex + SparkFlex, CAN 20, 50A)
 *  - Pre-Shooter (NEO Vortex + SparkFlex, CAN 21, 40A)
 *  - Shooter     (2x NEO Vortex + SparkFlex, CAN 22 + 23, 80A each)
 */
public class ShooterSubsystem extends SubsystemBase {

    // Motors
    private final SparkMax  m_agitatorMotor          = new SparkMax(ShooterConstants.kAgitatorMotorID,         MotorType.kBrushless);
    private final SparkFlex m_kickerMotor             = new SparkFlex(ShooterConstants.kKickerMotorID,          MotorType.kBrushless);
    private final SparkFlex m_preShooterMotor         = new SparkFlex(ShooterConstants.kPreShooterMotorID,      MotorType.kBrushless);
    private final SparkFlex m_shooterPrimaryMotor     = new SparkFlex(ShooterConstants.kShooterPrimaryMotorID,  MotorType.kBrushless);
    private final SparkFlex m_shooterSecondaryMotor   = new SparkFlex(ShooterConstants.kShooterSecondaryMotorID, MotorType.kBrushless);

    // Closed-loop controllers
    private final SparkClosedLoopController m_agitatorController;
    private final SparkClosedLoopController m_kickerController;
    private final SparkClosedLoopController m_preShooterController;
    private final SparkClosedLoopController m_shooterPrimaryController;
    private final SparkClosedLoopController m_shooterSecondaryController;

    // Preferences keys for dashboard-tunable RPMs
    private static final String kShooterRPMKey = "Shooter/Target RPM";
    private static final String kPreShooterRPMKey = "Shooter/PreShooter Target RPM";
    private static final String kVerboseTelemetryKey = "Shooter/Verbose Telemetry";

    public ShooterSubsystem() {
        // Initialize Preferences with defaults (only writes if key doesn't already exist)
        Preferences.initDouble(kShooterRPMKey, ShooterConstants.kShooterForwardRPM);
        Preferences.initDouble(kPreShooterRPMKey, ShooterConstants.kPreShooterForwardRPM);
        Preferences.initBoolean(kVerboseTelemetryKey, false);
        // --- Agitator Motor (NEO + SparkMax) ---
        // outputRange(0,1): prevents active braking when the P-controller overshoots target RPM.
        // Motor coasts back to target instead of oscillating. Same fix applied to kicker.
        SparkMaxConfig agitatorConfig = new SparkMaxConfig();
        agitatorConfig
            .inverted(ShooterConstants.kAgitatorInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kAgitatorCurrentLimitAmps);
        agitatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kAgitatorP)
            .i(ShooterConstants.kAgitatorI)
            .d(ShooterConstants.kAgitatorD)
            .outputRange(0, 1);
        agitatorConfig.closedLoop.feedForward
            .kV(ShooterConstants.kAgitatorFF);
        REVUtil.check(
            m_agitatorMotor.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters),
            "Agitator motor (SparkMax ID " + ShooterConstants.kAgitatorMotorID + ") configure");
        m_agitatorController = m_agitatorMotor.getClosedLoopController();

        // --- Kicker Motor (NEO Vortex + SparkFlex) ---
        // High-P bang-bang style PID: outputRange(0,1) means it only drives forward.
        // When above target RPM the output is clamped to 0 so the motor coasts — no active
        // braking that would cause overshoot/undershoot oscillation.
        SparkFlexConfig kickerConfig = new SparkFlexConfig();
        kickerConfig
            .inverted(ShooterConstants.kKickerInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kKickerCurrentLimitAmps);
        kickerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kKickerP)
            .i(ShooterConstants.kKickerI)
            .d(ShooterConstants.kKickerD)
            .outputRange(0, 1);
        kickerConfig.closedLoop.feedForward
            .kV(ShooterConstants.kKickerFF);
        REVUtil.check(
            m_kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters),
            "Kicker motor (SparkFlex ID " + ShooterConstants.kKickerMotorID + ") configure");
        m_kickerController = m_kickerMotor.getClosedLoopController();

        // --- Pre-Shooter Motor (NEO Vortex + SparkFlex) ---
        // High-P onboard PID (acts like bang-bang at 1kHz on motor controller)
        // No feedforward — P alone drives full power when below target, coasts when above
        // Previous settings: .p(0.0002), .kV(1.0/6784.0)
        // Previous roboRIO bang-bang: full power below target, coast above (20ms loop)
        SparkFlexConfig preShooterConfig = new SparkFlexConfig();
        preShooterConfig
            .inverted(ShooterConstants.kPreShooterInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kPreShooterCurrentLimitAmps);
        preShooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kShooterBangBangP)
            .i(0.0)
            .d(0.0)
            .outputRange(0, 1);
        preShooterConfig.closedLoop.feedForward
            .kV(ShooterConstants.kPreShooterFF);
        REVUtil.check(
            m_preShooterMotor.configure(preShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters),
            "Pre-shooter motor (SparkFlex ID " + ShooterConstants.kPreShooterMotorID + ") configure");
        m_preShooterController = m_preShooterMotor.getClosedLoopController();

        // --- Shooter Primary Motor (NEO Vortex + SparkFlex, CAN 22) ---
        // High-P onboard PID (bang-bang style at 1kHz)
        SparkFlexConfig shooterPrimaryConfig = new SparkFlexConfig();
        shooterPrimaryConfig
            .inverted(ShooterConstants.kShooterPrimaryInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kShooterCurrentLimitAmps);
        shooterPrimaryConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kShooterBangBangP)
            .i(0.0)
            .d(0.0)
            .outputRange(0, 1);
        shooterPrimaryConfig.closedLoop.feedForward
            .kV(ShooterConstants.kShooterFF);
        REVUtil.check(
            m_shooterPrimaryMotor.configure(shooterPrimaryConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters),
            "Shooter primary motor (SparkFlex ID " + ShooterConstants.kShooterPrimaryMotorID + ") configure");
        m_shooterPrimaryController = m_shooterPrimaryMotor.getClosedLoopController();

        // --- Shooter Secondary Motor (NEO Vortex + SparkFlex, CAN 23) ---
        // High-P onboard PID (bang-bang style at 1kHz)
        SparkFlexConfig shooterSecondaryConfig = new SparkFlexConfig();
        shooterSecondaryConfig
            .inverted(ShooterConstants.kShooterSecondaryInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kShooterCurrentLimitAmps);
        shooterSecondaryConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kShooterBangBangP)
            .i(0.0)
            .d(0.0)
            .outputRange(0, 1);
        shooterSecondaryConfig.closedLoop.feedForward
            .kV(ShooterConstants.kShooterFF);
        REVUtil.check(
            m_shooterSecondaryMotor.configure(shooterSecondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters),
            "Shooter secondary motor (SparkFlex ID " + ShooterConstants.kShooterSecondaryMotorID + ") configure");
        m_shooterSecondaryController = m_shooterSecondaryMotor.getClosedLoopController();
    }

    // -------------------------------------------------------------------------
    // Agitator
    // -------------------------------------------------------------------------

    /** Runs the agitator motor at the configured forward RPM using closed-loop velocity control. */
    public void runAgitator() {
        m_agitatorController.setSetpoint(ShooterConstants.kAgitatorForwardRPM, ControlType.kVelocity);
    }

    /** Runs the agitator motor at a custom RPM.
     *  Positive RPM uses closed-loop velocity control (outputRange [0,1]).
     *  Negative RPM uses open-loop duty cycle because outputRange(0,1) clamps closed-loop to zero.
     */
    public void runAgitatorAtRPM(double rpm) {
        if (rpm < 0) {
            // Open-loop reverse: scale by NEO free speed (5676 RPM) to get approximate duty cycle
            m_agitatorMotor.set(rpm / 5676.0);
        } else {
            m_agitatorController.setSetpoint(rpm, ControlType.kVelocity);
        }
    }

    /** Stops the agitator motor. */
    public void stopAgitator() {
        m_agitatorMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Kicker
    // -------------------------------------------------------------------------

    /** Runs the kicker motor at the configured forward RPM using closed-loop velocity control. */
    public void runKicker() {
        m_kickerController.setSetpoint(ShooterConstants.kKickerForwardRPM, ControlType.kVelocity);
    }

    /** Runs the kicker motor slowly in reverse during shooter spin-up to prevent pre-loading. */
    public void runKickerSlowReverse() {
        // Use open-loop duty cycle for reverse — closed-loop outputRange is clamped to [0,1]
        // for forward operation; a fixed slow duty cycle is sufficient for pre-load prevention.
        m_kickerMotor.set(-0.08);  // ~8% reverse — just enough to resist pre-loading
    }

    /** Stops the kicker motor. */
    public void stopKicker() {
        m_kickerMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Pre-Shooter
    // -------------------------------------------------------------------------

    /** Runs the pre-shooter at the dashboard-tunable RPM using high-P onboard PID. */
    public void runPreShooter() {
        runPreShooterAtRPM(Preferences.getDouble(kPreShooterRPMKey, ShooterConstants.kPreShooterForwardRPM));
    }

    /** Runs the pre-shooter at the given RPM using the onboard PID controller. */
    public void runPreShooterAtRPM(double rpm) {
        m_preShooterController.setSetpoint(rpm, ControlType.kVelocity);
    }

    /** Stops the pre-shooter motor. */
    public void stopPreShooter() {
        m_preShooterMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Shooter
    // -------------------------------------------------------------------------

    /** Runs both shooter flywheels at the dashboard-tunable RPM using high-P onboard PID. */
    public void runShooter() {
        runShooterAtRPM(Preferences.getDouble(kShooterRPMKey, ShooterConstants.kShooterForwardRPM));
    }

    /** Runs both shooter flywheels at the given RPM using the onboard PID controller. */
    public void runShooterAtRPM(double rpm) {
        m_shooterPrimaryController.setSetpoint(rpm, ControlType.kVelocity);
        m_shooterSecondaryController.setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Adjusts the Preferences-stored default shooter RPM by {@code delta} RPM,
     * clamped to [{@link ShooterConstants#kManualShootMinRPM}, {@link ShooterConstants#kManualShootMaxRPM}].
     *
     * <p>Called by D-pad Up/Down bindings on the operator controller.
     *
     * @param delta Amount to add (positive = faster, negative = slower).
     */
    public void adjustDefaultRPM(double delta) {
        double current = Preferences.getDouble(kShooterRPMKey, ShooterConstants.kShooterForwardRPM);
        double next = Math.max(ShooterConstants.kManualShootMinRPM,
                      Math.min(ShooterConstants.kManualShootMaxRPM, current + delta));
        Preferences.setDouble(kShooterRPMKey, next);
    }

    /** Stops both shooter flywheel motors. */
    public void stopShooter() {
        m_shooterPrimaryMotor.stopMotor();
        m_shooterSecondaryMotor.stopMotor();
    }

    // -------------------------------------------------------------------------
    // Combined
    // -------------------------------------------------------------------------

    /** Runs all 4 stages simultaneously. */
    public void runAll() {
        runAgitator();
        runKicker();
        runPreShooter();
        runShooter();
    }

    /** Runs all 4 stages simultaneously in reverse for clearing jams. */
    public void reverseAll() {
        // All motors with outputRange(0,1) cannot accept negative closed-loop setpoints —
        // PID output is clamped to 0. Use open-loop duty cycle for all reverse operations.
        m_agitatorMotor.set(-ShooterConstants.kReverseAllDutyCycle);
        m_kickerMotor.set(-ShooterConstants.kReverseAllDutyCycle);
        m_preShooterMotor.set(-ShooterConstants.kReverseAllDutyCycle);
        m_shooterPrimaryMotor.set(-ShooterConstants.kReverseAllDutyCycle);
        m_shooterSecondaryMotor.set(-ShooterConstants.kReverseAllDutyCycle);
    }

    /** Stops all 4 stages (5 motors). */
    public void stopAll() {
        stopAgitator();
        stopKicker();
        stopPreShooter();
        stopShooter();
    }

    /**
     * Returns true when the pre-shooter motor's encoder velocity has reached or exceeded
     * {@link ShooterConstants#kPreShooterKickerThresholdRPM}, indicating it is spinning
     * fast enough for the kicker to begin feeding.
     */
    public boolean isPreShooterAboveKickerThreshold() {
        return m_preShooterMotor.getEncoder().getVelocity() >= ShooterConstants.kPreShooterKickerThresholdRPM;
    }

    /**
     * Returns true if both shooter flywheel motors are within the tolerance
     * window of the dashboard-tunable target RPM.
     */
    public boolean isShooterAtSpeed() {
        double targetRPM = Preferences.getDouble(kShooterRPMKey, ShooterConstants.kShooterForwardRPM);
        double primaryVelocity = m_shooterPrimaryMotor.getEncoder().getVelocity();
        double secondaryVelocity = m_shooterSecondaryMotor.getEncoder().getVelocity();
        return Math.abs(primaryVelocity - targetRPM) <= ShooterConstants.kShooterSpeedToleranceRPM
            && Math.abs(secondaryVelocity - targetRPM) <= ShooterConstants.kShooterSpeedToleranceRPM;
    }

    /**
     * Returns true if both shooter flywheel motors are within tolerance of the given target RPM.
     *
     * @param targetRPM The target RPM to check against.
     */
    public boolean isShooterAtSpeed(double targetRPM) {
        double primary   = m_shooterPrimaryMotor.getEncoder().getVelocity();
        double secondary = m_shooterSecondaryMotor.getEncoder().getVelocity();
        return Math.abs(primary   - targetRPM) <= ShooterConstants.kShooterSpeedToleranceRPM
            && Math.abs(secondary - targetRPM) <= ShooterConstants.kShooterSpeedToleranceRPM;
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        double targetRPM   = Preferences.getDouble(kShooterRPMKey, ShooterConstants.kShooterForwardRPM);
        double primaryRPM  = m_shooterPrimaryMotor.getEncoder().getVelocity();
        double secondaryRPM = m_shooterSecondaryMotor.getEncoder().getVelocity();

        // Always-on: the values you need to tune RPM setpoints
        SmartDashboard.putBoolean("Shooter/AtSpeed",          isShooterAtSpeed());
        SmartDashboard.putNumber("Shooter/Commanded RPM",     targetRPM);
        SmartDashboard.putNumber("Shooter/Primary RPM",       primaryRPM);
        SmartDashboard.putNumber("Shooter/Secondary RPM",     secondaryRPM);
        SmartDashboard.putNumber("Shooter/Primary RPM Error", targetRPM - primaryRPM);
        SmartDashboard.putNumber("Shooter/Secondary RPM Error", targetRPM - secondaryRPM);

        if (Preferences.getBoolean(kVerboseTelemetryKey, false)) {
            SmartDashboard.putNumber("Shooter/Agitator RPM",   m_agitatorMotor.getEncoder().getVelocity());
            SmartDashboard.putNumber("Shooter/Kicker RPM",     m_kickerMotor.getEncoder().getVelocity());
            SmartDashboard.putNumber("Shooter/PreShooter RPM", m_preShooterMotor.getEncoder().getVelocity());
            SmartDashboard.putBoolean("Shooter/PreShooter AtSpeed",
                m_preShooterMotor.getEncoder().getVelocity()
                    >= Preferences.getDouble(kPreShooterRPMKey, ShooterConstants.kPreShooterForwardRPM));
            SmartDashboard.putNumber("Shooter/Commanded PreShooter RPM",
                Preferences.getDouble(kPreShooterRPMKey, ShooterConstants.kPreShooterForwardRPM));
        }
    }
}
