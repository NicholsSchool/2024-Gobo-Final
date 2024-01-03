package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//TODO: modify if needed for smoothing turning input

/**
 * A trapezoid Profiler for smoothing changing values
 */
public class TrapezoidProfile {
    /** The default initial value */
    public static final double DEFAULT_INITIAL_VALUE = 0.0;

    /** The default minimum value */
    public static final double DEFAULT_MIN_VALUE = -1.0;

    /** The default maximum value */
    public static final double DEFAULT_MAX_VALUE = 1.0;

    /** The default maximum value change per millisecond */
    public static final double DEFAULT_MAX_CHANGE_PER_MILLI = 0.005;

    private final ElapsedTime timer;
    private final double minValue;
    private final double maxValue;
    private final double maxDelta;
    private double previousValue;


    /**
     * Instantiates the Profile with the default values
     */
    public TrapezoidProfile() {
        this(DEFAULT_INITIAL_VALUE, DEFAULT_MIN_VALUE, DEFAULT_MAX_VALUE, DEFAULT_MAX_CHANGE_PER_MILLI);
    }

    /**
     * Instantiates the Profile with the specified caps
     *
     * @param initialValue the starting value
     * @param min the minimum value
     * @param max the maximum value
     * @param maxChange the maximum change
     */
    public TrapezoidProfile(double initialValue, double min, double max, double maxChange) {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        previousValue = initialValue;
        minValue = min;
        maxValue = max;
        maxDelta = maxChange;
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

        double result;
        if(slope > maxDelta)
             result = Range.clip(previousValue + maxDelta * time, minValue, maxValue);
        else if(slope < -maxDelta)
            result = Range.clip(previousValue - maxDelta * time, minValue, maxValue);
        else
            result = Range.clip(newValue, minValue, maxValue);

        previousValue = result;
        timer.reset();
        return result;
    }
}