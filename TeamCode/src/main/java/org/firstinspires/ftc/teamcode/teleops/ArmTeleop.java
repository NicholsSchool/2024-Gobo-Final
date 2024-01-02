package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//TODO: further tune goToPos, wrist fourbar (with 2 positions), sync right shoulder with dead wheel

/**
 * Arm Testing
 */
@Config
@TeleOp(name="[DASHBOARD] Arm Teleop")
public class ArmTeleop extends OpMode {
    /** Arm Proportional Constant */
    public static double SHOULDER_P = 0.0005;

    /** Arm Scaling Factor Constant */
    public static double SHOULDER_F = 0.000022;

    /** Arm Derivative Constant */
    public static double SHOULDER_D = 0.0000044;

    /** Shoulder Maximum Power */
    public static final double SHOULDER_GOVERNOR = 0.25;

    /** Shoulder Maximum Power */
    public static final double SHOULDER_GOVERNOR_DOWN = 0.01;

    /** The Vertical Encoder Position for the arm */
    public static final int ARM_VERTICAL_POSITION = 2850;

    private ElapsedTime loopTimer;
    private DcMotorEx leftShoulder;
    private DcMotorEx rightShoulder;
    private int previousPosition;
    public static double power;
    public static boolean goToPos;
    public static int desiredPosition;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftShoulder = hardwareMap.get(DcMotorEx.class, "leftShoulder");
        leftShoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShoulder.setDirection(DcMotorEx.Direction.REVERSE);

        rightShoulder = hardwareMap.get(DcMotorEx.class, "rightShoulder");
        rightShoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShoulder.setDirection(DcMotorEx.Direction.REVERSE); //TODO: sync with dead wheel

        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(goToPos)
            armGoToPosition(desiredPosition);
        else
            shoulderManual(power);

        previousPosition = getArmPosition();

        telemetry.addData("arm position", previousPosition);
        telemetry.addData("loop time millis", loopTimer.time());
        telemetry.update();

        loopTimer.reset();
    }

    /**
     * Moves both shoulder motors together manually
     *
     * @param power the input motor power [-1, 1]
     */
    public void shoulderManual(double power) {
        power = Range.clip(power, -SHOULDER_GOVERNOR, SHOULDER_GOVERNOR);
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
        double power = SHOULDER_P * (desiredPosition - position);
        double scalingFactor = SHOULDER_F * (ARM_VERTICAL_POSITION - position);
        double dampening = SHOULDER_D * (position - previousPosition);

        double output = power + scalingFactor - dampening;
        if( dampening != 0.0 && ((dampening < 0.0) == (position <= ARM_VERTICAL_POSITION)))
            output = Range.clip(output, -SHOULDER_GOVERNOR_DOWN, SHOULDER_GOVERNOR_DOWN);

        shoulderManual(output);
    }

}
