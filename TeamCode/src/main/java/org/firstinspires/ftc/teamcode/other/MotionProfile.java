package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.ProfileConstants;

/**
 * A simple Motion Profiler for smoothing changes to a value
 */
public class MotionProfile implements ProfileConstants {
    private final ElapsedTime timer;
    private final double minValue;
    private final double maxValue;
    private final double maxSpeed;
    private double previousValue;


    /**
     * Instantiates the Profile with the default values
     */
    public MotionProfile() {
        this(0.0, -MAX, MAX, MAX_SPEED);
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

        if(change > maxChange)
            previousValue = Range.clip(previousValue + maxChange, minValue, maxValue);
        else if(change < -maxChange)
            previousValue = Range.clip(previousValue - maxChange, minValue, maxValue);
        else
            previousValue = Range.clip(newValue, minValue, maxValue);

        return previousValue;
    }
}