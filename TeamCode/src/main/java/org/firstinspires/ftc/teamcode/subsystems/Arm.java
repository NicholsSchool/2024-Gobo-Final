package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.other.Constants;

//TODO: fix and tune arm go to position and governor
//TODO: tune wrist go to position and governor manually
//TODO: fiddle with the fourbar switching angle

/**
 * Robot Arm Subsystem
 */
public class Arm {
    private final DcMotorEx leftShoulder;
    private final DcMotorEx rightShoulder;
    private final DcMotorEx wrist;
    private final Servo planeLauncher;
    private final ElapsedTime timer;
    private double previousArmPosition;

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

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    /**
     * Updates data for arm go to position control
     */
    public void update() {
        previousArmPosition = getArmPosition();
        timer.reset();
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
        double dampening = 0.0000044 * (position - previousArmPosition) / timer.time();

        shoulderManual(power + scalingFactor - dampening);
    }

    /**
     * Moves the wrist motor manually
     *
     * @param power the input motor power
     */
    public void wristManual(double power) {
        wrist.setPower(Range.clip(power, -0.025, 0.025));
    }

    /**
     * The wrist encoder position
     *
     * @return the position
     */
    public int getWristPosition() {
        return wrist.getCurrentPosition();
    }

    /**
     * Moves the wrist to the given position
     *
     * @param position the target encoder position
     */
    public void wristGoToPosition(int position) {
        wristManual(0.005 * (position - wrist.getCurrentPosition()));
    }

    /**
     * Moves the wrist to the intaking or scoring angle automatically
     */
    public void wristFourbar() {
        final int CORE_HEX_TICKS_PER_REV = 288;

        if(getArmPosition() <= 300)
            wristGoToPosition((int)Math.round(-getArmPosition() * CORE_HEX_TICKS_PER_REV * 1.0 / Constants.THRU_BORE_TICKS_PER_REV));
        else
            wristGoToPosition((int)Math.round(48 - getArmPosition() * CORE_HEX_TICKS_PER_REV * 1.0 / Constants.THRU_BORE_TICKS_PER_REV));
    }
}