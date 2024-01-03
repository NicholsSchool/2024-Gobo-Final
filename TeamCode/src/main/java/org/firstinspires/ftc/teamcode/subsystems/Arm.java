package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.Constants;

//TODO: sync right shoulder direction with dead wheel
//TODO: fix and tune arm go to position and governor
//TODO: consider using arm angle conversion
//TODO: tune wrist go to position and governor, try using RUN_TO_POSITION run-mode or dampening
//TODO: tune fourbar switching angle

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
     * Moves the arm to position
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
     * @param power the input motor power [-1, 1]
     */
    public void wristManual(double power) {
        wrist.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Calculates the angle of the Wrist motor using its encoder position
     *
     * @return the angle
     */
    public double getWristAngle() {
        int CORE_HEX_TICKS_PER_REV = 288;
        return wrist.getCurrentPosition() * 360.0 / CORE_HEX_TICKS_PER_REV + 90.0;
    }

    /**
     * Moves the wrist to the given angle
     *
     * @param angle the desired angle
     */
    public void wristGoToAngle(double angle) {
        wristManual(0.01 * (angle - getWristAngle()));
    }

    /**
     * Moves the wrist to in-taking or scoring angle automatically
     */
    public void wristFourbar() {
        if(getArmPosition() <= 300)
            wristGoToAngle(-getArmPosition() * 360.0 / Constants.THRU_BORE_TICKS_PER_REV);
        else
            wristGoToAngle(60.0 - getArmPosition() * 360.0 / Constants.THRU_BORE_TICKS_PER_REV);
    }
}