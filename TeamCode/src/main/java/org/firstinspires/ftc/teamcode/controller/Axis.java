package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * An Axis on a Controller (triggers and joysticks)
 */
public class Axis {
    /** The default Joystick deadband */
    public static final double DEFAULT_DEADBAND = 0.005;

    /** The maximum value change per millisecond */
    public static final double MAX_CHANGE_PER_MILLI = 0.005;

    private final ElapsedTime timer;
    private double value;
    private final double deadband;

    /**
     * Instantiates an Axis with the default deadband
     */
    public Axis() {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        value = 0.0;
        deadband = DEFAULT_DEADBAND;
    }

    /**
     * Instantiates an Axis with the specified deadband
     */
    public Axis(double desiredDeadband) {
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        value = 0.0;
        deadband = desiredDeadband;
    }

    private double applyDeadband(double newValue) {
        return Math.abs(newValue) >= deadband ? newValue : 0.0;
    }

    private double trapezoidProfile(double newValue) {
        double slope = (newValue - value) / timer.time();
        timer.reset();

        if(slope > MAX_CHANGE_PER_MILLI)
            return Range.clip(value + MAX_CHANGE_PER_MILLI, -1.0, 1.0);
        if(slope < -MAX_CHANGE_PER_MILLI)
            return Range.clip(value - MAX_CHANGE_PER_MILLI, -1.0, 1.0);
        return newValue;
    }

    /**
     * Updates the Axis with the new state
     * Applies the deadband and Trapezoid Profile
     */
    public void update(double newValue) {
        value = trapezoidProfile(applyDeadband(newValue));
    }

    /**
     * The Axis value
     *
     * @return the value
     */
    public double getValue() {
        return value;
    }

    /**
     * The value as a String for telemetry
     *
     * @return the Axis value
     */
    @NonNull
    public String toString() {
        return String.valueOf(value);
    }
}
