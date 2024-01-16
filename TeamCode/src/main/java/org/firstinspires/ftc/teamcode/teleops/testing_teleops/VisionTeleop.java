package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * A teleop to copy paste edit with
 */
@Config
@TeleOp(name="[DASHBOARD] Vision Testing")
public class VisionTeleop extends OpMode {
    private ElapsedTime loopTimer;
    private Vision vision;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private double x;
    private double y;
    private double heading;


    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        x = 60;
        y = 60;
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        vision = new Vision(hardwareMap, x, y);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double[] pose = vision.update();

        if(pose != null) {
            x = pose[0];
            y = pose[1];
            heading = pose[2];
        }

        TelemetryPacket packet = new TelemetryPacket(true);
        packet.fieldOverlay()
                .setFill("green")
                .fillCircle(-x, -y, 14.0);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", heading);
        telemetry.addData("num detections", vision.getNumDetections());
        telemetry.addData("loop time millis", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
    }
}
