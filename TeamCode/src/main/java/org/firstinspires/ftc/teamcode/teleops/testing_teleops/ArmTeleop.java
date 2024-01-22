package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

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
    public static boolean climb;
    public static int armDesiredPosition;
    public static boolean wristGoToPos;
    public static double wristDesiredPos;
    public static double wristPower;
    public static boolean launchPlane;
    public static boolean isFloat;

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
        if(isFloat)
            arm.setFloat();

        if(climb)
            arm.climb(shoulderPower);
        else if(armGoToPos)
            arm.armGoToPosition(armDesiredPosition);
        else
            arm.armManual(shoulderPower);

        if(wristGoToPos)
            arm.wristGoToPos(wristDesiredPos);
        else
            arm.wristManual(wristPower);

        arm.launchPlane(launchPlane);

        telemetry.addData("arm position", arm.getArmPosition());
        telemetry.addData("wrist position", arm.getWristPosition());
        telemetry.addData("loop time millis", loopTimer.time());
        loopTimer.reset();

        telemetry.update();
    }
}
