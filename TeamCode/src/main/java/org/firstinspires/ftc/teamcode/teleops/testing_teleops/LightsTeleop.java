package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lights;

/**
 * Teleop for testing Lights functionalities
 */
@Config
@TeleOp(name="[DASHBOARD] Lights Testing")
public class LightsTeleop extends OpMode {
    private ElapsedTime loopTimer;
    private Lights lights;

    public static boolean isDefault;
    public static boolean leftGreen;
    public static boolean rightGray;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        lights = new Lights(hardwareMap, true);
        isDefault = true;

        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(isDefault)
            lights.setDefaultPattern();
        else {
            if(leftGreen)
                lights.setLeftPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            if(rightGray)
                lights.setRightPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
        }

        telemetry.addData("loop time millis", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
    }
}
