package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Sample Auto to Copy and Paste
 */
@Autonomous(name="Judging Room")
public class JudgingRoomAuto extends LinearOpMode implements TeleopConstants, DriveConstants {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, IS_BLUE_ALLIANCE, 0.0, 0.0, 90.0);
        drivetrain.setFloat();

        waitForStart();
        runtime.reset();

        double distance = SPLINE_ERROR;

        while (distance >= SPLINE_ERROR) {
            drivetrain.update();
            double[] pose = drivetrain.getXY();

            double[] xyInput = drivetrain.vectorToVertex(72.0, 72.0, true);

            distance = Math.hypot(72.0 - pose[0], 72.0 - pose[1]);
            double powerRatio = (distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0) / Math.hypot(xyInput[0], xyInput[1]);

            drivetrain.drive(xyInput[0] * powerRatio, xyInput[1] * powerRatio, 0.1, false, true);
        }

        double error = AUTO_ALIGN_ERROR;

        while (error >= AUTO_ALIGN_ERROR) {
            drivetrain.update();
            double angle = drivetrain.getFieldHeading();
            drivetrain.drive(0.0, 0.0, 0.0, true, true);
            error = Math.abs(angle - 90.0);
        }


        drivetrain.drive(0, 0, 0.25, false, false);
    }
}