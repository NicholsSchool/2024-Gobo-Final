package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * A simple Motion Profiler for smoothing changes to a value
 */
public class MotionProfile {
    private final ElapsedTime timer;
    private final double minValue;
    private final double maxValue;
    private final double maxSpeed;
    private double previousValue;


    /**
     * Instantiates the Profile with the default values
     */
    public MotionProfile() {
        this(0.0, -0.25, 0.25, 1.0);
    }

    /**
     * Instantiates the Profile with the specified values
     *
     * @param initialValue the starting value
     * @param min the minimum value
     * @param max the maximum value
     * @param maxSpeed the maximum change per second
     */
    public MotionProfile(double initialValue, double min, double max, double maxSpeed) {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        previousValue = initialValue;
        minValue = min;
        maxValue = max;
        this.maxSpeed = maxSpeed;
    }

    /**
     * Updates the Profile and returns the next value.
     * Call in each loop()
     *
     * @param newValue the new value to smooth
     *
     * @return the smoothed new value
     */
    public double update(double newValue) {
        double time = timer.time();
        timer.reset();

        double change = newValue - previousValue;

        double maxChange = maxSpeed * time;
        double result;
        if(change > maxChange)
             result = Range.clip(previousValue + maxChange, minValue, maxValue);
        else if(change < -maxChange)
            result = Range.clip(previousValue - maxChange, minValue, maxValue);
        else
            result = Range.clip(newValue, minValue, maxValue);

        previousValue = result;
        return result;
    }
}