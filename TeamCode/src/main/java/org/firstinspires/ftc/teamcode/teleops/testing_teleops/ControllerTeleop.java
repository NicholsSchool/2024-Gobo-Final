package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;

//TODO: put color over the grid, do the correct rotation and scaling, and center the square

/**
 * Teleop for testing Controller and Profiling functionalities
 */
@Config
@TeleOp(name="Controller Testing")
public class ControllerTeleop extends OpMode {
    private Controller driverController;
    private ElapsedTime loopTimer;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        driverController = new Controller(gamepad1);
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        driverController.update();

        double leftX = driverController.leftStickX.getValue();
        double leftY = driverController.leftStickY.getValue();
        telemetry.addData("left stick x", leftX);
        telemetry.addData("left stick y", leftY);

        telemetry.addData("right stick x", driverController.rightStickX);

        telemetry.addData("loop time millis", loopTimer.time());

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                .drawGrid(0.0, 0.0, 144.0, 144.0, 21, 21)
                .setFill("red")
                .fillRect(leftY * 72, -leftX * 72, 7.2, 7.2);
        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
        loopTimer.reset();
    }
}
