package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;

/**
 * An Axis on a Controller (triggers and joysticks)
 */
public class Axis {
    /** The default Joystick deadband */
    public static final double DEFAULT_DEADBAND = 0.005;
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
        value = 0.0;
        deadband = desiredDeadband;
    }

    /**
     * Updates the Axis with the new state using the deadband
     */
    public void update(double newValue) {
        value = Math.abs(newValue) >= deadband ? newValue : 0.0;
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
     * The value as a String for telemetry
     *
     * @return the Axis value
     */
    @NonNull
    public String toString() {
        return String.valueOf(value);
    }
}