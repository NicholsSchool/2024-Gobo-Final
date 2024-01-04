package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

//TODO: full check of all arm functionalities

/**
 * Teleop for testing Arm functionalities
 */
@Config
@TeleOp(name="[DASHBOARD] Arm Teleop")
public class ArmTeleop extends OpMode {
    private ElapsedTime loopTimer;
    private Arm arm;
    public static double shoulderPower;
    public static boolean armGoToPos;
    public static int armDesiredPosition;
    public static boolean isFourbar;
    public static boolean wristGoToPos;
    public static double wristPower;
    public static int wristDesiredPosition;
    public static boolean launchPlane;
    public static double p;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        arm = new Arm(hardwareMap);

        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(armGoToPos)
            arm.armGoToPosition(armDesiredPosition);
        else
            arm.shoulderManual(shoulderPower);

        if(isFourbar)
            arm.getWristPosition(); //TODO: put fourbar back
        else if(wristGoToPos)
            arm.wristGoToPosition(wristDesiredPosition, p);
        else
            arm.wristManual(wristPower);

        arm.launchPlane(launchPlane);

        arm.update();

        telemetry.addData("arm ticks", arm.getArmPosition());
        telemetry.addData("wrist position", arm.getWristPosition());
        telemetry.addData("loop time millis", loopTimer.time());
        telemetry.update();

        loopTimer.reset();
    }
}
