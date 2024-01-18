package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.other.CoordinateMotionProfile;
import org.firstinspires.ftc.teamcode.other.MotionProfile;

//TODO: put color over the grid, do the correct rotation and scaling, and center the square

/**
 * Teleop for testing Controller and Profiling functionalities
 */
@Config
@TeleOp(name="[DASHBOARD] Controller Testing")
public class ControllerTeleop extends OpMode {
    private Controller driverController;
    private ElapsedTime loopTimer;
    private CoordinateMotionProfile joystickProfile;
    private MotionProfile axisProfile;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static boolean clip;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        driverController = new Controller(gamepad1);
        joystickProfile = new CoordinateMotionProfile();
        axisProfile = new MotionProfile();

        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        driverController.update();

        double[] xy;
        if(clip)
            xy = joystickProfile.update(
                    driverController.leftStickX.getValue(), driverController.leftStickY.getValue(), 0.5);
        else
            xy = joystickProfile.update(
                    driverController.leftStickX.getValue(), driverController.leftStickY.getValue());

        telemetry.addData("left stick x", xy[0]);
        telemetry.addData("left stick y", xy[1]);

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                .drawGrid(0.0, 0.0, 144.0, 144.0, 21, 21)
                .setFill("red")
                .fillRect(xy[1] * 72, -xy[0] * 72, 7.2, 7.2);
        dashboard.sendTelemetryPacket(packet);

        double turn = axisProfile.update(driverController.rightStickX.getValue() * 0.25);
        telemetry.addData("right stick x", turn);

        telemetry.addData("loop time millis", loopTimer.time());
        loopTimer.reset();
        telemetry.update();}
}
