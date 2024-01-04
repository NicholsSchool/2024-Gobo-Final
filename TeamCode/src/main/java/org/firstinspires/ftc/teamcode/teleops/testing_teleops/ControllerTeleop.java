package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.utilities.CoordinateTrapezoidProfile;
import org.firstinspires.ftc.teamcode.utilities.TrapezoidProfile;

//TODO: make the field image better, do rotation, image, correct scaling, centered square, etc

/**
 * Teleop for testing Controller functionalities
 */
@Config
@TeleOp(name="Controller Testing")
public class ControllerTeleop extends OpMode {
    private Controller driverController;
    private ElapsedTime loopTimer;
    private TrapezoidProfile triggerProfile;
    private CoordinateTrapezoidProfile joystickProfile;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        driverController = new Controller(gamepad1);
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        triggerProfile = new TrapezoidProfile();
        joystickProfile = new CoordinateTrapezoidProfile();
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

        double[] leftJoystick = joystickProfile.update(driverController.rightStickX.getValue(), driverController.rightStickY.getValue());
        telemetry.addData("right stick x", leftJoystick[0]);
        telemetry.addData("right stick y", leftJoystick[1]);
        telemetry.addData("left stick radius", driverController.leftStickRadius());
        telemetry.addData("left stick theta", driverController.leftStickTheta());

        telemetry.addData("left trigger", driverController.leftTrigger);
        telemetry.addData("right trigger", triggerProfile.update(driverController.rightTrigger.getValue()));

        telemetry.addData("loop time millis", loopTimer.time());

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                .drawGrid(0.0, 0.0, 144.0, 144.0, 21, 21)
                .setFill("blue")
                .fillRect(leftJoystick[1] * 72, -leftJoystick[0] * 72, 7.2, 7.2);

        dashboard.sendTelemetryPacket(packet);

        telemetry.update();

        loopTimer.reset();
    }
}
