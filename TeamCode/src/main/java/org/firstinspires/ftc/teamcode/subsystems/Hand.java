package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Robot Hand Subsystem
 */
public class Hand {
    private final Servo leftGrabber;
    private final Servo rightGrabber;

    /**
     * Initializes the Hand
     */
    public Hand(HardwareMap hardwareMap) {
        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        leftGrabber.setDirection(Servo.Direction.FORWARD);
        leftGrabber.scaleRange(0.65, 0.825);

        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");
        rightGrabber.setDirection(Servo.Direction.FORWARD);
        rightGrabber.scaleRange(0.175, 0.35);
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