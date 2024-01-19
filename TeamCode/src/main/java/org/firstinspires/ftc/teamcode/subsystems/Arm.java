package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.ProfileConstants;
import org.firstinspires.ftc.teamcode.other.MotionProfile;

/**
 * Robot Arm Subsystem
 */
public class Arm implements ArmConstants, ProfileConstants {
    private final DcMotorEx leftShoulder;
    private final DcMotorEx rightShoulder;
    private final DcMotorEx wrist;
    private final Servo planeLauncher;
    private final MotionProfile climbProfile;
    private int encoderOffset;

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
        planeLauncher.scaleRange(ArmConstants.PLANE_MIN, ArmConstants.PLANE_MAX);

        climbProfile = new MotionProfile(0.0, -CLIMB_MAX, CLIMB_MAX, CLIMB_MAX_SPEED);
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
        power = Range.clip(power, -SHOULDER_MAX, SHOULDER_MAX);
        leftShoulder.setPower(power);
        rightShoulder.setPower(power);
    }

    /**
     * Shoulder input for climbing with no power governor
     *
     * @param power the input motor power
     */
    public void climb(double power) {
        power = climbProfile.update(power);
        leftShoulder.setPower(power);
        rightShoulder.setPower(power);
    }

    /**
     * The angle of the arm measured in thru bore ticks
     *
     * @return the encoder position of the arm
     */
    public int getArmPosition() {
        return leftShoulder.getCurrentPosition() - encoderOffset;
    }

    /**
     * Moves the arm to position using an external feedback loop
     *
     * @param desiredPosition the encoder position to go to
     */
    public void armGoToPosition(double desiredPosition) {
        double position = getArmPosition();
        shoulderManual(SHOULDER_P * (desiredPosition - position) +
                SHOULDER_F * Math.cos(Math.PI * position / ARM_VERTICAL));
    }

    public int getWristPosition(){
        return wrist.getCurrentPosition();
    }

    public void wristToPosition(int desiredPosition){
        int position = getWristPosition();
        wristManual(SHOULDER_P * 5 * (desiredPosition - position));
    }

    /**
     * Moves the wrist motor manually
     *
     * @param power the input motor power
     */
    public void wristManual(double power) {
        wrist.setPower(Range.clip(power, -WRIST_MAX, WRIST_MAX));
    }

    /**
     * Sets the arm to Float mode
     */
    public void setFloat() {
        leftShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Resets the arm encoder. Use when the arm is in the down position
     */
    public void resetEncoder() {
        encoderOffset = leftShoulder.getCurrentPosition();
    }
}