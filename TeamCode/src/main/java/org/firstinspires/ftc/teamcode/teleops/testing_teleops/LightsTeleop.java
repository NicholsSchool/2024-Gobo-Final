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

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        lights = new Lights(hardwareMap, true);

        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(isDefault)
            lights.setAllianceColor();
        else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        }

        telemetry.addData("loop time millis", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
    }
}
