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
        boolean lowGear = driverController.leftTrigger.getValue() >= 0.5;

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

        double x = driverController.leftStickX.getValue();
        double y = driverController.leftStickY.getValue();
        double turn = driverController.rightStickX.getValue();
        drivetrain.drive(x, y, turn, autoAlign, lowGear);

        double[] xy = drivetrain.getXY();
        telemetry.addData("robot x", xy[0]);
        telemetry.addData("robot y", xy[1]);
        telemetry.addData("theta", drivetrain.getFieldHeading());

        telemetry.addData("loop time millis", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
    }
}
