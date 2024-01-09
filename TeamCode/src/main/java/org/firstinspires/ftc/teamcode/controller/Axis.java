package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * An Axis on a Controller (triggers and joysticks)
 */
public class Axis {
    /** The default Joystick deadband */
    public static final double DEFAULT_DEADBAND = 0.005;
    private final ElapsedTime timer;
    private double value;
    private final double deadband;

    /**
     * Instantiates an Axis with the default deadband
     */
    public Axis() {
        this(DEFAULT_DEADBAND);
    }

    /**
     * Instantiates an Axis with the specified deadband
     */
    public Axis(double desiredDeadband) {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        value = 0.0;
        deadband = desiredDeadband;
    }

    /**
     * Updates the Axis with the new state using the deadband
     */
    public void update(double newValue) {
        value = Math.abs(newValue) >= deadband ? newValue : 0.0;
        if(value != 0.0)
            timer.reset();
    }

    /**
     * The current Axis value
     *
     * @return the value
     */
    public double getValue() {
        return value;
    }

    /**
     * Whether the Axis value has been 0 for the minimum time interval
     *
     * @return iff the timer is over the interval
     */
    public boolean zeroLongEnough() {
        return timer.time() >= 0.5;
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