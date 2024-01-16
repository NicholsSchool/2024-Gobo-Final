package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.ProfileConstants;

/**
 * A Motion Profile for a Point on a Coordinate Plane
 */
public class CoordinateMotionProfile implements ProfileConstants {
    private final ElapsedTime timer;
    private final double minValue;
    private final double maxValue;
    private final double maxSpeed;
    private double previousX;
    private double previousY;

    /**
     * Instantiates the Profile with the default values
     */
    public CoordinateMotionProfile() {
        this(0.0, 0.0, -COORDINATE_MAX, COORDINATE_MAX, COORDINATE_MAX_SPEED);
    }

    /**
     * Instantiates the Profile with the specified caps
     *
     * @param initialX the starting X value
     * @param initialY the starting Y value
     * @param min the minimum value
     * @param max the maximum value
     * @param maxSpeed the maximum change per second
     */
    public CoordinateMotionProfile(double initialX, double initialY, double min, double max, double maxSpeed) {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        previousX = initialX;
        previousY = initialY;
        minValue = min;
        maxValue = max;
        this.maxSpeed = maxSpeed;
    }

    /**
     * Updates the Profile and returns the next value.
     * Call in each loop()
     *
     * @param newX the new X value
     * @param newY the new Y value
     *
     * @return the smoothed coordinate [x, y]
     */
    public double[] update(double newX, double newY) {
        double time = timer.time();
        timer.reset();

        double change = Math.hypot(newX - previousX, newY - previousY);
        double maxChange = maxSpeed * time;

        if(change <= maxChange) {
            previousX = Range.clip(newX, minValue, maxValue);
            previousY = Range.clip(newY, minValue, maxValue);
        }
        else {
            double ratio = maxChange / change;
            previousX = Range.clip(previousX + (newX - previousX) * ratio, minValue, maxValue);
            previousY = Range.clip(previousY + (newY - previousY) * ratio, minValue, maxValue);
        }

        return new double[]{previousX, previousY};
    }

    /**
     * Updates the Profile and returns the next value.
     * Call in each loop()
     *
     * @param newX the new X value
     * @param newY the new Y value
     * @param tempMaxDist the temporary Max distance between the output (x, y) and (0, 0)
     *
     * @return the smoothed new coordinates
     */
    public double[] update(double newX, double newY, double tempMaxDist) {
        double[] result = update(newX, newY);

        double distance = Math.hypot(result[0], result[1]);

        if(distance > tempMaxDist) {
            double ratio = tempMaxDist / distance;
            result[0] *= ratio;
            result[1] *= ratio;
        }

        previousX = result[0];
        previousY = result[1];

        return result;
    }
}
