package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.ProfileConstants;
import org.firstinspires.ftc.teamcode.other.MotionProfile;

/**
 * Robot Arm Subsystem
 */
public class Arm implements ArmConstants, ProfileConstants, DriveConstants {
    private final DcMotorEx leftShoulder;
    private final DcMotorEx rightShoulder;
    private final DcMotorEx wrist;
    private final Servo planeLauncher;
    private final MotionProfile climbProfile;
    private double armDesiredPosition;
    private double wristDesiredPosition;
    private int armOffset;
    private int wristOffset;

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

        climbProfile = new MotionProfile(0.0, -CLIMB_MAX, 0.0, CLIMB_MAX_SPEED);
    }

    /**
     * Brings the Servo back to load position
     */
    public void loadPlane() {
        planeLauncher.setPosition(0.0);
    }

    /**
     * Launches the plane using the Servo
     */
    public void launchPlane() {
        planeLauncher.setPosition(1.0);
    }

    /**
     * Moves both shoulder motors together manually
     *
     * @param power the input motor power
     */
    public void armManual(double power) {
        climbProfile.update(power);

        power = Range.clip(power, -SHOULDER_MAX, SHOULDER_MAX);
        leftShoulder.setPower(power);
        rightShoulder.setPower(power);
    }

    /**
     * Shoulder input for climbing
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
        return leftShoulder.getCurrentPosition() - armOffset;
    }

    /**
     * Moves the arm to position using an external feedback loop
     */
    public void armGoToPosition() {
        double position = getArmPosition();
        armManual(SHOULDER_P * (armDesiredPosition - position) +
                SHOULDER_F * Math.cos(Math.PI * position / ARM_VERTICAL));
    }

    /**
     * Sets the arm desired position
     *
     * @param position the thru bore encoder position
     */
    public void setDesiredArmPosition(double position) {
        armDesiredPosition = position;
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
     * The Wrist Encoder Position
     *
     * @return the Core Hex Encoder Ticks
     */
    public int getWristPosition() {
        return wrist.getCurrentPosition() - wristOffset;
    }

    /**
     * Moves the wrist motor to the encoder position
     */
    public void wristGoToPos() {
        wristManual(WRIST_P * (wristDesiredPosition - getWristPosition()));
    }

    /**
     * Sets the wrist desired position
     *
     * @param position the thru bore encoder position
     */
    public void setDesiredWristPosition(double position) {
        wristDesiredPosition = position;
    }

    /**
     * Moves the wrist to the intaking or scoring angle automatically
     */
    public void wristFourbar() {
        if(getArmPosition() <= WRIST_SWITCHING_POS)
            setDesiredWristPosition(WRIST_DOWN - getArmPosition());
        else
            setDesiredWristPosition(WRIST_UP - getArmPosition());
        wristGoToPos();
    }

    /**
     * Sets the arm to Float mode
     */
    public void setFloat() {
        leftShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Offsets the arm and wrist encoders. Use when arm and wrist are fully down
     */
    public void resetEncoders() {
        armOffset = leftShoulder.getCurrentPosition();
        wristOffset = wrist.getCurrentPosition();
    }
}