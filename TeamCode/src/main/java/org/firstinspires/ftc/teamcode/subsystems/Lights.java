package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

//TODO: store useful patterns as constants, make methods w/o params. Use for displaying on major actions

/**
 * Robot Indicator Lights Subsystem
 * A full list of REV blink codes can be found
 * <a href="https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html">here</a>.
 */
public class Lights {
    private final RevBlinkinLedDriver leftBlinkin;
    private final RevBlinkinLedDriver rightBlinkin;
    private final BlinkinPattern defaultPattern;

    /**
     * Initializes the Lights subsystem
     *
     * @param hwMap the hardwareMap
     * @param isBlueAlliance whether we are blue alliance
     */
    public Lights(HardwareMap hwMap, boolean isBlueAlliance) {
        leftBlinkin = hwMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        rightBlinkin = hwMap.get(RevBlinkinLedDriver.class, "rightBlinkin");

        defaultPattern = isBlueAlliance ? BlinkinPattern.BLUE : BlinkinPattern.RED;
        setPattern(defaultPattern);
    }

    /** Sets both left and right LED strips to a certain color pattern.
     *
     * @param pattern The BlinkinPattern to set the LEDs to
     */
    public void setPattern(BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
        rightBlinkin.setPattern(pattern);
    }

    /**
     * Sets left LED strip to a certain color pattern
     *
     * @param pattern The BlinkinPattern to set the LED to
     */
    public void setLeftPattern(BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
    }

    /**
     * Sets right LED strip to a certain color pattern
     *
     * @param pattern The BlinkinPattern to set the LED to
     */
    public void setRightPattern(BlinkinPattern pattern) {
        rightBlinkin.setPattern(pattern);
    }

    /**
     * Sets the default color based on alliance
     */
    public void setDefaultPattern() {
        leftBlinkin.setPattern(defaultPattern);
        rightBlinkin.setPattern(defaultPattern);
    }
}