package org.firstinspires.ftc.teamcode.other;

/**
 * Angles are dumb. Insert methods to take care of those dumb cases
 */
public class AngleMath {
    /**
     * Adds angles and puts the sum in the range [-180, 180)
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
