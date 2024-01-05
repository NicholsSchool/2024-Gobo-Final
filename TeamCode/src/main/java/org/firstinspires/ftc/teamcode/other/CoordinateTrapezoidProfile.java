package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * A Profile for Smoothing Changing Coordinates
 */
public class CoordinateTrapezoidProfile {
    private final ElapsedTime timer;
    private final double minValue;
    private final double maxValue;
    private final double maxSpeed;
    private double previousX;
    private double previousY;


    /**
     * Instantiates the Profile with the default values
     */
    public CoordinateTrapezoidProfile() {
        this(0.0, 0.0, -1.0, 1.0, 4.0);
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
    public CoordinateTrapezoidProfile(double initialX, double initialY, double min, double max, double maxSpeed) {
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
        double change = Math.hypot(newX - previousX, newY - previousY);

        double[] result;
        if(change <= maxSpeed * time)
            result = new double[]{
                    Range.clip(newX, minValue, maxValue),
                    Range.clip(newY, minValue, maxValue)};
        else
            result = new double[]{
                    Range.clip(previousX + maxSpeed * time * (newX - previousX) / change,
                            minValue, maxValue),
                    Range.clip(previousY + maxSpeed * time * (newY - previousY) / change,
                            minValue, maxValue)};

        previousX = result[0];
        previousY = result[1];

        timer.reset();
        return result;
    }
}
