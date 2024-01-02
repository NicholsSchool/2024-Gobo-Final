package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Robot Arm Subsystem
 */
public class Arm {
    /** Arm Proportional Constant */
    public static final double ARM_P = 0.0001;

    /** Shoulder Maximum Power */
    public static final double SHOULDER_GOVERNOR = 1.0;

    private final DcMotorEx leftShoulder;
    private final DcMotorEx rightShoulder;
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

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        planeLauncher.setDirection(Servo.Direction.FORWARD);
        planeLauncher.scaleRange(0.0, 1.0); //TODO: check range
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

    public void planeLauncherManual(double position) {
        //TODO: replace with boolean parameter method
        planeLauncher.setPosition(position);
    }

    /**
     * The angle of the arm measured in thru bore ticks
     *
     * @return the encoder position of the arm
     */
    public int getPosition() {
        return leftShoulder.getCurrentPosition();
    }

    /**
     * Moves the arm to position
     *
     * @param position the thru bore encoder position
     */
    public void goToPosition(int position) {
        //TODO: more sophisticated motion control
        shoulderManual(ARM_P * (position - getPosition()));
    }
}