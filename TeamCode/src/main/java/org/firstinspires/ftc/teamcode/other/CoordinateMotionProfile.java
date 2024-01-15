package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.other.Constants.ProfileConstants;

/**
 * A Motion Profile for a Point on a Coordinate Plane
 */
public class CoordinateMotionProfile {
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
        this(0.0, 0.0, -ProfileConstants.COORDINATE_MAX,
                ProfileConstants.COORDINATE_MAX, ProfileConstants.COORDINATE_MAX_SPEED);
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
     * @return the smoothed new coordinates
     */
    public double[] update(double newX, double newY) {
        double time = timer.time();
        timer.reset();

        double change = Math.hypot(newX - previousX, newY - previousY);
        double maxChange = maxSpeed * time;

        double[] result;
        if(change <= maxChange)
            result = new double[]{
                    Range.clip(newX, minValue, maxValue),
                    Range.clip(newY, minValue, maxValue)};
        else
            result = new double[]{
                    Range.clip(previousX + maxChange * (newX - previousX) / change,
                            minValue, maxValue),
                    Range.clip(previousY + maxChange * (newY - previousY) / change,
                            minValue, maxValue)};

        previousX = result[0];
        previousY = result[1];

        return result;
    }

    /**
     * Updates the Profile and returns the next value.
     * Call in each loop()
     *
     * @param newX the new X value
     * @param newY the new Y value
     * @param tempMaxDist the temporary Maximum distance between the output x and y and (0, 0)
     *
     * @return the smoothed new coordinates
     */
    public double[] update(double newX, double newY, double tempMaxDist) {
        double[] result = update(newX, newY);

        double distance = Math.hypot(result[0], result[1]);
        if(distance > tempMaxDist) {
            result[0] *= tempMaxDist / distance;
            result[1] *= tempMaxDist / distance;
        }

        previousX = result[0];
        previousY = result[1];

        return result;
    }
}
