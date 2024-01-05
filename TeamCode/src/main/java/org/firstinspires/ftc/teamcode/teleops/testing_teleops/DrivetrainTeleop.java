package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.other.Constants;
import org.firstinspires.ftc.teamcode.other.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

//TODO: Test full drivetrain functionalities

/**
 * A teleop to copy paste edit with
 */
@Config
@TeleOp(name="[DASHBOARD] Drivetrain Testing")
public class DrivetrainTeleop extends OpMode {
    private ElapsedTime loopTimer;
    private Drivetrain drivetrain;
    private TrapezoidProfile speedSmoothing;

    public static double spinSpeed;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, Constants.IS_BLUE_ALLIANCE, 0.0, 0.0, 90.0);
        speedSmoothing = new TrapezoidProfile();

        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drivetrain.update();

        drivetrain.simpleSpin(speedSmoothing.update(spinSpeed));

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

        telemetry.addData("heading", drivetrain.getFieldHeading());

        telemetry.addData("loop time millis", loopTimer.time());
        telemetry.update();

        loopTimer.reset();
    }
}
