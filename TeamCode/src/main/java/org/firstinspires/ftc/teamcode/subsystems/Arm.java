package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//TODO: switch out arm functionalities for the one in the teleop

/**
 * Robot Arm Subsystem
 */
public class Arm {
    /** Arm Proportional Constant */
    public static final double SHOULDER_P = 0.0001;

    /** Shoulder Maximum Power */
    public static final double SHOULDER_GOVERNOR = 1.0;

    /** Wrist Proportional Constant */
    public static final double WRIST_P = 0.01;

    /** Wrist Maximum Power */
    public static final double WRIST_GOVERNOR = 1.0;

    /** Wrist Encoder ticks per full revolution */
    private static final int WRIST_TICKS_PER_REV = 288;

    /** Starting angle offset of the wrist */
    private static final double WRIST_STARTING_OFFSET = 90.0;

    private final DcMotorEx leftShoulder;
    private final DcMotorEx rightShoulder;
    private final DcMotorEx wrist;
    private final Servo planeLauncher;

    /**
     * Initializes the Arm
     */
    public Arm(HardwareMap hardwareMap) {
        leftShoulder = hardwareMap.get(DcMotorEx.class, "leftShoulder");
        leftShoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShoulder.setDirection(DcMotorEx.Direction.FORWARD); //TODO: sync with thru bore

        rightShoulder = hardwareMap.get(DcMotorEx.class, "rightShoulder");
        rightShoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShoulder.setDirection(DcMotorEx.Direction.FORWARD); //TODO: sync with dead wheels

        wrist = hardwareMap.get(DcMotorEx.class, "wrist");
        wrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wrist.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wrist.setDirection(DcMotorEx.Direction.REVERSE);

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        planeLauncher.setDirection(Servo.Direction.FORWARD);
        planeLauncher.scaleRange(0.35, 1.0);
    }

    /**
     * Launches the plane using the Servo
     *
     * @param isLaunching whether to go to launch or cocked position
     */
    public void launchPlane(boolean isLaunching) {
        planeLauncher.setPosition(isLaunching ? 1.0 : 0.0);
    }

    /**
     * Moves both shoulder motors together manually
     *
     * @param power the input motor power [-1, 1]
     */
    public void shoulderManual(double power) {
        power = Range.clip(power, -SHOULDER_GOVERNOR, SHOULDER_GOVERNOR);
        //TODO: check directions
        leftShoulder.setPower(power);
        rightShoulder.setPower(power);
    }

    /**
     * The angle of the arm measured in thru bore ticks
     *
     * @return the encoder position of the arm
     */
    public int getArmPosition() {
        return leftShoulder.getCurrentPosition();
    }

    /**
     * Moves the arm to position
     *
     * @param position the thru bore encoder position
     */
    public void armGoToPosition(int position) {
        //TODO: more sophisticated motion control
        shoulderManual(SHOULDER_P * (position - getArmPosition()));
    }

    /**
     * Moves the wrist motor manually
     *
     * @param power the input motor power [-1, 1]
     */
    public void wristManual(double power) {
        wrist.setPower(Range.clip(power, -WRIST_GOVERNOR, WRIST_GOVERNOR));
    }

    /**
     * Calculates the angle of the Wrist motor using its encoder position
     *
     * @return the angle
     */
    public double getWristAngle() {
        return wrist.getCurrentPosition() * 360.0 / WRIST_TICKS_PER_REV + WRIST_STARTING_OFFSET;
    }

    /**
     * Moves the wrist to the given angle
     *
     * @param angle the desired angle
     */
    public void wristGoToAngle(double angle) {
        wristManual(WRIST_P * (angle - getWristAngle()));
    }
}