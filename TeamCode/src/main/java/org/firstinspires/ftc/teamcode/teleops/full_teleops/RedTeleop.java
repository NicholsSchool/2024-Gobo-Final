package org.firstinspires.ftc.teamcode.teleops.full_teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.other.TeleopRobot;

/**
 * A teleop to copy paste edit with
 */
@TeleOp(name="Red Competition")
public class RedTeleop extends OpMode implements TeleopConstants {
    private TeleopRobot robot;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new TeleopRobot(hardwareMap, IS_RED_ALLIANCE, gamepad1, gamepad2, telemetry, new double[]{0.0, 0.0, -180.0});
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.update();
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
