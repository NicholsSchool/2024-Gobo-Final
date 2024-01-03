package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//TODO: check for bugs

/**
 * A trapezoid Profiler for smoothing changing values
 */
public class TrapezoidProfile {
    private final ElapsedTime timer;
    private final double minValue;
    private final double maxValue;
    private final double maxDeltaPerSecond;
    private double previousValue;


    /**
     * Instantiates the Profile with the default values
     */
    public TrapezoidProfile() {
        this(0.0, -1.0, 1.0, 2.0);
    }

    /**
     * Instantiates the Profile with the specified caps
     *
     * @param initialValue the starting value
     * @param min the minimum value
     * @param max the maximum value
     * @param maxChange the maximum change per second
     */
    public TrapezoidProfile(double initialValue, double min, double max, double maxChange) {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        previousValue = initialValue;
        minValue = min;
        maxValue = max;
        maxDeltaPerSecond = maxChange;
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
        double slope = (newValue - previousValue) / time;

        double maxDelta = maxDeltaPerSecond * time;
        double result;
        if(slope > maxDelta)
             result = Range.clip(previousValue + maxDelta, minValue, maxValue);
        else if(slope < -maxDelta)
            result = Range.clip(previousValue - maxDelta, minValue, maxValue);
        else
            result = Range.clip(newValue, minValue, maxValue);

        previousValue = result;

        timer.reset();
        return result;
    }
}