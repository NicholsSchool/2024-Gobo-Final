package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.other.Spline;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name = "Bezier Auto version 1")
public class SampleBezierAuto extends LinearOpMode implements DriveConstants {

    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {
        ElapsedTime sampleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, true, 36, -65, 0.0);

        double[][] points1 = new double[][]{{36, -65}, {72.4, 25.3}, {-51.7, -0.8}, {-49.3, -37}};
        double[][] points2 = new double[][]{{-38, -58}, {85.3, -2.6}, {0.7, 25.7}, {-21.8, -6.5}};

        Spline spline1 = new Spline(points1, 20, drivetrain, 100);
        Spline spline2 = new Spline(points2, 20, drivetrain, 100);
        spline1.update();
        spline2.update();

        waitForStart();

        double[] scanCoords = new double[]{36.0, -36.0};
        double distance = SPLINE_ERROR;
        while(distance >= SPLINE_ERROR) {
            drivetrain.update();
            double[] pose = drivetrain.getXY();

            double[] xyInput = drivetrain.vectorToVertex(scanCoords[0], scanCoords[1], true);

            distance = Math.hypot(scanCoords[0] - pose[0], scanCoords[1] - pose[1]);
            double powerRatio = (distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0) / Math.hypot(xyInput[0], xyInput[1]);

            drivetrain.drive(xyInput[0] * powerRatio, xyInput[1] * powerRatio, 0.0, true, true);
        }

        while(spline1.desiredT() < 0.97) {
            spline1.update();
            spline2.update();
            double[] robotPose = drivetrain.getXY();
            double power1 = Range.clip(
                    SPLINE_P * Math.hypot(robotPose[0] - points1[3][0], robotPose[1] - points1[3][1]), 0.0, 1.0);
            double turn = 0;
            boolean autoAlign = true;
            if(spline1.desiredT() > 0.6){
                drivetrain.setDesiredHeading(-180);
            }


            drivetrain.drive(power1 * Math.cos(spline1.angle()), power1 * Math.sin(spline1.angle()), turn, autoAlign, false);
            if(sampleTime.time() > 30) {
                spline1.update();
                sampleTime.reset();
            }
            telemetry.addData("x", drivetrain.getXY()[0]);
            telemetry.addData("y", drivetrain.getXY()[1]);
            telemetry.addData("t", spline1.desiredT());
            telemetry.addData("bezierX", spline1.bezierX(spline1.desiredT()));
            telemetry.addData("bezierY", spline1.bezierY(spline1.desiredT()));
            telemetry.addData("t2", (spline2.desiredT()));
            telemetry.update();
        }
//        spline2.update();
//        while (spline2.desiredT() < 0.95) {
//            spline2.update();
//            double power2 = 0.7 * Range.clip(1.4 - spline2.desiredT(), 0, 1);
//
//            drivetrain.drive(power2 * Math.cos(spline2.angle()), power2 * Math.sin(spline2.angle()), 0, false, false);
//            if (sampleTime.time() > 30) {
//                spline2.update();
//                sampleTime.reset();
//            }
//            telemetry.addData("x", drivetrain.getXY()[0]);
//            telemetry.addData("y", drivetrain.getXY()[1]);
//            telemetry.addData("t", spline2.desiredT());
//            telemetry.addData("bezierX", spline2.bezierX(spline2.desiredT()));
//            telemetry.addData("bezierY", spline2.bezierY(spline2.desiredT()));
//            telemetry.update();
//        }
        drivetrain.drive(0, 0, 0, false, false);
    }
}