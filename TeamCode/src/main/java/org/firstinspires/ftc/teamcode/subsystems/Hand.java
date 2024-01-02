package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Robot Hand Subsystem
 */
public class Hand {
    /** Wrist Proportional Constant */
    public static final double WRIST_P = 0.025;

    /** Wrist Maximum Power */
    public static final double WRIST_GOVERNOR = 1.0;

    /** Wrist Encoder ticks per full revolution */
    private static final int WRIST_TICKS_PER_REV = 288;

    /** Starting angle offset of the wrist */
    private static final double WRIST_OFFSET = 100.0;

    private final DcMotorEx wrist;
    private final Servo leftGrabber;
    private final Servo rightGrabber;

    /**
     * Initializes the Hand
     */
    public Hand(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(DcMotorEx.class, "wrist");
        wrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wrist.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wrist.setDirection(DcMotorEx.Direction.FORWARD); //TODO: check direction

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        leftGrabber.setDirection(Servo.Direction.FORWARD); //TODO: check direction
        leftGrabber.scaleRange(0.0, 1.0); //TODO: check range

        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");
        rightGrabber.setDirection(Servo.Direction.FORWARD); //TODO: check direction
        rightGrabber.scaleRange(0.0, 1.0); //TODO: check range
    }

    /**
     * Moves the wrist motor manually
     *
     * @param power the input motor power [-1, 1]
     */
    public void wristManual(double power) {
        wrist.setPower(Range.clip(power, -WRIST_GOVERNOR, WRIST_GOVERNOR));
    }

    public void leftManual(double position) {
        //TODO: replace with boolean parameter method
        leftGrabber.setPosition(position);
    }

    public void rightManual(double position) {
        //TODO: replace with boolean parameter method
        rightGrabber.setPosition(position);
    }

    /**
     * Calculates the angle of the Wrist motor using its encoder position
     *
     * @return the angle
     */
    public double getWristAngle() {
        return wrist.getCurrentPosition() * 360.0 / WRIST_TICKS_PER_REV + WRIST_OFFSET;
    }

    /**
     * Moves the arm to position
     *
     * @param position the thru bore encoder position
     */
    public void goToPosition(int position) {
        //TODO: add a dampening control?
        wristManual(WRIST_P * (position - getWristAngle()));
    }
}