package frc.robot.util;

import frc.robot.Constants.ShooterTableConstants;

/**
 * Utility class for interpolating shooter RPM from a distance-to-RPM lookup table.
 *
 * <p>Performs linear interpolation between table breakpoints and clamps to the
 * nearest endpoint if distance is outside the table's range.
 */
public final class ShooterInterpolation {

    private ShooterInterpolation() {}

    /**
     * Interpolates shooter flywheel RPM for the given distance to the hub.
     *
     * @param distanceMeters Distance from robot center to hub center (meters).
     * @return Interpolated shooter RPM.
     */
    public static double getShooterRPM(double distanceMeters) {
        return interpolate(distanceMeters, ShooterTableConstants.kColShooterRPM);
    }

    /**
     * Interpolates pre-shooter RPM for the given distance to the hub.
     *
     * @param distanceMeters Distance from robot center to hub center (meters).
     * @return Interpolated pre-shooter RPM.
     */
    public static double getPreShooterRPM(double distanceMeters) {
        return interpolate(distanceMeters, ShooterTableConstants.kColPreShooterRPM);
    }

    private static double interpolate(double distanceMeters, int rpmColumn) {
        double[][] table = ShooterTableConstants.kShooterTable;
        int distCol = ShooterTableConstants.kColDistance;

        // Clamp below minimum
        if (distanceMeters <= table[0][distCol]) {
            return table[0][rpmColumn];
        }

        // Clamp above maximum
        if (distanceMeters >= table[table.length - 1][distCol]) {
            return table[table.length - 1][rpmColumn];
        }

        // Find the two surrounding rows and interpolate
        for (int i = 0; i < table.length - 1; i++) {
            double d0 = table[i][distCol];
            double d1 = table[i + 1][distCol];
            if (distanceMeters >= d0 && distanceMeters <= d1) {
                double t = (distanceMeters - d0) / (d1 - d0);
                return table[i][rpmColumn] + t * (table[i + 1][rpmColumn] - table[i][rpmColumn]);
            }
        }

        // Should never reach here if table is sorted
        return table[table.length - 1][rpmColumn];
    }
}
