package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//TODO: tune arm go to position and governor
//TODO: tune wrist governor

/**
 * Robot Arm Subsystem
 */
public class Arm {
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
        leftShoulder.setDirection(DcMotorEx.Direction.REVERSE);

        rightShoulder = hardwareMap.get(DcMotorEx.class, "rightShoulder");
        rightShoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShoulder.setDirection(DcMotorEx.Direction.REVERSE);

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
     * @param power the input motor power
     */
    public void shoulderManual(double power) {
        power = Range.clip(power, -0.5, 0.5);
        leftShoulder.setPower(power);
        rightShoulder.setPower(power);
    }

    /**
     * Climbs at Max Shoulder Power
     */
    public void climb() {
        leftShoulder.setPower(1.0);
        rightShoulder.setPower(1.0);
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
     * Moves the arm to position using an external feedback loop
     *
     * @param desiredPosition the encoder position to go to
     */
    public void armGoToPosition(double desiredPosition) {
        double position = getArmPosition();

        double power = 0.0005 * (desiredPosition - position);
        double scalingFactor = 0.000022 * (2850 - position);

        shoulderManual(power + scalingFactor);
    }

    /**
     * Moves the wrist motor manually
     *
     * @param power the input motor power
     */
    public void wristManual(double power) {
        wrist.setPower(Range.clip(power, -0.5, 0.5));
    }
}