package org.firstinspires.ftc.teamcode.utilities;

/**
 * Methods for doing math with Angles
 */
public class AngleMath {
    //TODO: april tag localization math can go here, including weighted averaging based on distance

    /**
     * Adds angles and keeps the sum in the range [-180, 180)
     *
     * @param angle1 the first angle
     * @param angle2 the second angle
     * @return the sum in degrees
     */
    public static double addAngles(double angle1, double angle2) {
        double sum = angle1 + angle2;

        while(sum >= 180.0)
            sum -= 360.0;
        while(sum < -180.0)
            sum += 360.0;

        return sum;
    }
}
