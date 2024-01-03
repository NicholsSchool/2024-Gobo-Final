package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;

//TODO: full check of all controller functionalities

/**
 * Teleop for testing Controller functionalities
 */
@Config
@TeleOp(name="Controller Testing")
public class ControllerTeleop extends OpMode {
    Controller driverController;
    ElapsedTime loopTimer;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        driverController = new Controller(gamepad1);
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        driverController.update();

        telemetry.addData("a is pressed", driverController.a);
        telemetry.addData("b toggle", driverController.b.toggleState());
        telemetry.addData("x just pressed", driverController.x.wasJustPressed());

        telemetry.addData("left trigger", driverController.leftTrigger);
        telemetry.addData("right stick y", driverController.rightStickY);
        telemetry.addData("right stick x", driverController.rightStickX);
        telemetry.addData("left stick radius", driverController.leftStickRadius());
        telemetry.addData("left stick theta", driverController.leftStickTheta());

        telemetry.addData("loop time millis", loopTimer.time());
        telemetry.update();

        loopTimer.reset();
    }
}
