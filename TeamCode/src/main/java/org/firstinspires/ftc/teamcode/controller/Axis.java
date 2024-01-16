package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.ControllerConstants;
import org.firstinspires.ftc.teamcode.constants.ProfileConstants;

/**
 * An Axis on a Controller
 */
public class Axis implements ControllerConstants, ProfileConstants {
    private final ElapsedTime timer;
    private final double deadband;
    private double value;

    /**
     * Instantiates the Axis
     */
    public Axis() {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        deadband = AXIS_DEADBAND;
    }

    /**
     * Updates the Axis with the new state applied through the deadband
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
     * Whether the Axis value has been zero for the minimum time interval
     *
     * @return whether the timer is over the interval
     */
    public boolean zeroLongEnough() {
        return timer.time() >= MAX * MAX_SPEED + ZERO_WAIT;
    }

    /**
     * The value as a String for telemetry
     *
     * @return the Axis value
     */
    @NonNull
    public String toString() {
        return "" + value;
    }
}