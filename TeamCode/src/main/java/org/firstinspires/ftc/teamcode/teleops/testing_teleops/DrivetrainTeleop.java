package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.other.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

//TODO: Test full drivetrain functionalities

/**
 * A teleop to copy paste edit with
 */
@Config
@TeleOp(name="[DASHBOARD] Drivetrain Testing")
public class DrivetrainTeleop extends OpMode {
    private ElapsedTime loopTimer;
    private Controller driverController;
    private Drivetrain drivetrain;
    public static double spinSpeed;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        driverController = new Controller(gamepad1);
        drivetrain = new Drivetrain(hardwareMap, Constants.IS_BLUE_ALLIANCE, 0.0, 0.0, 90.0);

        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        driverController.update();
        drivetrain.update();

        boolean autoAlign = driverController.rightStickX.zeroLongEnough();

        if(!autoAlign)
            drivetrain.setDesiredHeading(drivetrain.getFieldHeading());
        else if(driverController.y.wasJustPressed())
            drivetrain.setDesiredHeading(90.0);
        else if(driverController.a.wasJustPressed())
            drivetrain.setDesiredHeading(-90.0);
        else if(driverController.b.wasJustPressed())
            drivetrain.setDesiredHeading(0.0);
        else if(driverController.x.wasJustPressed())
            drivetrain.setDesiredHeading(-180.0);

        double[] powerAngle = driverController.leftStick();
        double turn = driverController.rightStickX.getValue();
        drivetrain.drive(powerAngle[0], powerAngle[1], turn, autoAlign);

        telemetry.addData("power", powerAngle[0]);
        telemetry.addData("angle", powerAngle[1]);
        telemetry.addData("turn", turn);
        telemetry.addData("autoAlign", autoAlign);

        double[] velocities = drivetrain.getMotorVelocities();
        telemetry.addData("left vel", velocities[0]);
        telemetry.addData("right vel", velocities[1]);
        telemetry.addData("back vel", velocities[2]);

        double[] odometry = drivetrain.getOdometryPositions();
        telemetry.addData("raw left pos", odometry[0]);
        telemetry.addData("raw right pos", odometry[1]);
        telemetry.addData("raw front pos", odometry[2]);

        double[] xy = drivetrain.getXY();
        telemetry.addData("x", xy[0]);
        telemetry.addData("y", xy[1]);

        telemetry.addData("theta", drivetrain.getFieldHeading());

        telemetry.addData("loop time millis", loopTimer.time());
        telemetry.update();

        loopTimer.reset();
    }
}
