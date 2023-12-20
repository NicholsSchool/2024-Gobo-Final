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

    private double applyDeadband(double newValue) {
        return Math.abs(newValue) >= deadband ? newValue : 0.0;
    }

    /**
     * Updates the Axis with the new state
     * Applies the deadband and Trapezoid Profile
     */
    public void update(double newValue) {
        value = applyDeadband(newValue);
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