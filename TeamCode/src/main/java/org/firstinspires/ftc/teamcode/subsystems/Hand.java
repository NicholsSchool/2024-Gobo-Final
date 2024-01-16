package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HandConstants;

/**
 * Robot Hand Subsystem
 */
public class Hand implements HandConstants {
    private final Servo leftGrabber;
    private final Servo rightGrabber;

    /**
     * Initializes the Hand
     */
    public Hand(HardwareMap hardwareMap) {
        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        leftGrabber.setDirection(Servo.Direction.FORWARD);
        leftGrabber.scaleRange(HandConstants.LEFT_IN, LEFT_OUT);

        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");
        rightGrabber.setDirection(Servo.Direction.FORWARD);
        rightGrabber.scaleRange(HandConstants.RIGHT_OUT, RIGHT_IN);
    }

    /**
     * Controls the left grabber
     *
     * @param isClosing whether to open or close
     */
    public void leftGrabber(boolean isClosing) {
        leftGrabber.setPosition(isClosing ? 1.0 : 0.0);
    }

    /**
     * Controls the right grabber
     *
     * @param isClosing whether to open or close
     */
    public void rightGrabber(boolean isClosing) {
        rightGrabber.setPosition(isClosing ? 0.0 : 1.0);
    }
}