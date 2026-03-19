package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

/**
 * Simulates swerve drive module positions and gyro heading by integrating
 * commanded ChassisSpeeds over time. Used in Robot.simulationPeriodic().
 */
public class DriveSimulation {

    private static final double kDtSeconds = 0.02; // 20ms loop

    private double m_headingRadians = 0.0;
    private final double[] m_moduleDistancesMeters = new double[4];
    private final Rotation2d[] m_moduleAngles = new Rotation2d[4];

    public DriveSimulation() {
        for (int i = 0; i < 4; i++) {
            m_moduleAngles[i] = new Rotation2d();
        }
    }

    /**
     * Advances the simulation by one timestep using the given chassis speeds.
     */
    public void update(ChassisSpeeds speeds) {
        // Integrate heading
        m_headingRadians += speeds.omegaRadiansPerSecond * kDtSeconds;

        // Convert to module states
        SwerveModuleState[] states =
            SwerveConstants.kSwerveKinematics.toSwerveModuleStates(speeds);

        // Integrate each module's distance
        for (int i = 0; i < 4; i++) {
            m_moduleDistancesMeters[i] += states[i].speedMetersPerSecond * kDtSeconds;
            m_moduleAngles[i] = states[i].angle;
        }
    }

    /** Returns the simulated gyro heading. */
    public Rotation2d getHeading() {
        return new Rotation2d(m_headingRadians);
    }

    /** Returns the simulated module positions. */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = new SwerveModulePosition(m_moduleDistancesMeters[i], m_moduleAngles[i]);
        }
        return positions;
    }

    /** Resets the simulation state. */
    public void reset() {
        m_headingRadians = 0.0;
        for (int i = 0; i < 4; i++) {
            m_moduleDistancesMeters[i] = 0.0;
            m_moduleAngles[i] = new Rotation2d();
        }
    }
}
