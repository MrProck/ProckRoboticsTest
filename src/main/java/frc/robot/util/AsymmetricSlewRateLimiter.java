package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/**
 * A slew rate limiter with separate acceleration and deceleration rates.
 *
 * <p>"Acceleration" means the input magnitude is increasing (speeding up).
 * "Deceleration" means the input magnitude is decreasing (slowing down).
 *
 * <p>This is useful for axes where hard braking is dangerous (e.g., the
 * forward/backward axis on a robot with a narrow wheelbase that tips easily).
 */
public class AsymmetricSlewRateLimiter {

    private final double m_accelRate; // max change per second when speeding up
    private final double m_decelRate; // max change per second when slowing down
    private double m_lastOutput;
    private double m_lastTimestamp;

    /**
     * Creates a new AsymmetricSlewRateLimiter.
     *
     * @param accelRate Maximum rate of change (units/sec) when magnitude is increasing.
     * @param decelRate Maximum rate of change (units/sec) when magnitude is decreasing.
     */
    public AsymmetricSlewRateLimiter(double accelRate, double decelRate) {
        m_accelRate = accelRate;
        m_decelRate = decelRate;
        m_lastOutput = 0.0;
        m_lastTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    /**
     * Filters the input, applying the appropriate rate limit.
     *
     * @param input The input value to filter.
     * @return The rate-limited output.
     */
    public double calculate(double input) {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double dt = now - m_lastTimestamp;
        m_lastTimestamp = now;

        // "Decelerating" means the commanded magnitude is shrinking toward zero.
        // This intentionally uses magnitude only (not sign) so that releasing the
        // stick to zero — the primary tipping scenario — always uses the slower
        // decel rate, regardless of which direction the robot was travelling.
        boolean decelerating = Math.abs(input) < Math.abs(m_lastOutput);
        double rate = decelerating ? m_decelRate : m_accelRate;
        double maxChange = rate * dt;

        double delta = input - m_lastOutput;
        m_lastOutput += MathUtil.clamp(delta, -maxChange, maxChange);
        return m_lastOutput;
    }

    /**
     * Resets the limiter to the given value with no rate limiting applied.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_lastOutput = value;
        m_lastTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
}
