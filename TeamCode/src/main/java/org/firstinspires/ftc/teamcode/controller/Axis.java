package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.other.Constants.ControllerConstants;
import org.firstinspires.ftc.teamcode.other.Constants.ProfileConstants;

/**
 * An Axis on a Controller
 */
public class Axis {
    private final ElapsedTime timer;
    private final double deadband;
    private double value;

    /**
     * Instantiates the Axis
     */
    public Axis() {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        deadband = ControllerConstants.AXIS_DEADBAND;
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
     * Whether the Axis value has been 0 for the minimum time interval
     *
     * @return iff the timer is over the interval
     */
    public boolean zeroLongEnough() {
        return timer.time() >=
                ProfileConstants.MAX * ProfileConstants.MAX_SPEED + ControllerConstants.ZERO_WAIT;
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