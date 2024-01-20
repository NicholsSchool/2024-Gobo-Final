package org.firstinspires.ftc.teamcode.other;

import java.util.Arrays;

/**
 * Methods for doing math with Angles
 */
public class AngleMath {
    /**
     * Adds angles and sets the sum to the range [-180, 180)
     *
     * @param angle1 the first angle in degrees
     * @param angle2 the second angle in degrees
     * @return the sum in degrees
     */
    public static double addAnglesDegrees(double angle1, double angle2) {
        double sum = angle1 + angle2;

        while(sum >= 180.0)
            sum -= 360.0;
        while(sum < -180.0)
            sum += 360.0;

        return sum;
    }

    /**
     * Adds angles and sets the sum to the range [-pi, pi)
     *
     * @param angle1 the first angle in radians
     * @param angle2 the second angle in radians
     * @return the sum in degrees
     */
    public static double addAnglesRadians(double angle1, double angle2) {
        double sum = angle1 + angle2;

        while(sum >= Math.PI)
            sum -= 2 * Math.PI;
        while(sum < -Math.PI)
            sum += 2 * Math.PI;

        return sum;
    }

    public static int mode(Integer[] n)
    {
        Arrays.sort(n);

        int count2 = 0;
        int count1 = 0;
        int popular1 = 0;
        int popular2 = 0;


        for (int i = 0; i < n.length; i++)
        {
            popular1 = n[i];
            count1 = 1;

            for (int j = i + 1; j < n.length; j++)
            {
                if (popular1 == n[j]) count1++;
            }

            if (count1 > count2)
            {
                popular2 = popular1;
                count2 = count1;
            }

            else if(count1 == count2)
            {
                popular2 = Math.min(popular2, popular1);
            }
        }

        return popular2;
    }

}
