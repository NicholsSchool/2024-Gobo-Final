package org.firstinspires.ftc.teamcode.utilities;

//TODO: check for bugs

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * A Profile for Smoothing Changing Coordinates
 */
public class CoordinateTrapezoidProfile {
    private final ElapsedTime timer;
    private final double minValue;
    private final double maxValue;
    private final double maxDeltaPerSecond;
    private double previousX;
    private double previousY;


    /**
     * Instantiates the Profile with the default values
     */
    public CoordinateTrapezoidProfile() {
        this(0.0, 0.0, -1.0, 1.0, 1.0);
    }

    /**
     * Instantiates the Profile with the specified caps
     *
     * @param initialX the starting X value
     * @param initialY the starting Y value
     * @param min the minimum value
     * @param max the maximum value
     * @param maxChange the maximum change
     */
    public CoordinateTrapezoidProfile(double initialX, double initialY, double min, double max, double maxChange) {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        previousX = initialX;
        previousY = initialY;
        minValue = min;
        maxValue = max;
        maxDeltaPerSecond = maxChange;
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
        double delta = Math.hypot(newX - previousX, newY - previousY);

        double[] result;
        if(delta <= maxDeltaPerSecond * time)
            result = new double[]{
                    Range.clip(newX, minValue, maxValue),
                    Range.clip(newY, minValue, maxValue)};
        else
            result = new double[]{
                    Range.clip(previousX + maxDeltaPerSecond * time * (newX - previousX) / delta,
                            minValue, maxValue),
                    Range.clip(previousY + maxDeltaPerSecond * time * (newY - previousY) / delta,
                            minValue, maxValue)};

        previousX = result[0];
        previousY = result[1];

        timer.reset();
        return result;
    }
}
