package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Hand;

/**
 * Hand Testing
 */
@Config
@TeleOp(name="[DASHBOARD] Hand Testing")
public class HandTeleop extends OpMode {
    private ElapsedTime loopTimer;
    private Hand hand;
    public static boolean sync;
    public static boolean closeLeft;
    public static boolean closeRight;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        hand = new Hand(hardwareMap);
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        hand.leftGrabber(closeLeft);
        hand.rightGrabber(sync ? closeLeft : closeRight);

        telemetry.addData("loop time millis", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
    }
}