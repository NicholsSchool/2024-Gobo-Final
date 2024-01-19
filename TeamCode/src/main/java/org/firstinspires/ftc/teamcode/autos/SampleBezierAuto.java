package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.other.Spline;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name = "Bezier Auto version 1")
public class SampleBezierAuto extends LinearOpMode implements DriveConstants, ArmConstants {

    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {
        ElapsedTime sampleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime waitTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        Drivetrain drivetrain = new Drivetrain(hardwareMap, true, 36, -65, 90.0);
        Arm arm = new Arm(hardwareMap);
        Hand hand = new Hand(hardwareMap);

        hand.leftGrabber(true);
        hand.rightGrabber(true);

        Vision vision = new Vision(hardwareMap, 36.0, -65.0);
        Lights lights = new Lights(hardwareMap, true);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        double[][] points1 = new double[][]{{36, -65}, {72.4, 25.3}, {-29.8, 1.3}, {-40, -36.2}};
        double[][] points2 = new double[][]{{-38, -58}, {85.3, -2.6}, {0.7, 25.7}, {-21.8, -6.5}};

        Spline spline1 = new Spline(points1, 20, drivetrain, 100);
        Spline spline2 = new Spline(points2, 20, drivetrain, 100);
        spline1.update();
        spline2.update();

        waitForStart();

        double[] scanCoords = new double[]{36.0, -40.0};
        double distance = SPLINE_ERROR;

        drivetrain.setDesiredHeading(0.0);

        while (distance >= SPLINE_ERROR) {
            drivetrain.update();
            double[] pose = drivetrain.getXY();

            double[] xyInput = drivetrain.vectorToVertex(scanCoords[0], scanCoords[1], true);

            distance = Math.hypot(scanCoords[0] - pose[0], scanCoords[1] - pose[1]);
            double powerRatio = (distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0) / Math.hypot(xyInput[0], xyInput[1]);

            drivetrain.drive(xyInput[0] * powerRatio, xyInput[1] * powerRatio, 0.0, true, true);
        }

        drivetrain.drive(0, 0, 0, false, false);
        waitTime.reset();
        while (waitTime.time() < 1) {
        }

        boolean hasSeenAprilTag = false;
        ElapsedTime visionTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while (!hasSeenAprilTag && visionTimer.time() <= 1.0) {
            double[] visionPose = vision.update();
            if (visionPose != null) {
                drivetrain.setPose(visionPose);
                hasSeenAprilTag = true;

                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);

            }
        }

        while(spline1.desiredT() < 0.97) {
            spline1.update();
            spline2.update();
            double[] robotPose = drivetrain.getXY();

            hand.leftGrabber(true);
            hand.rightGrabber(true);

            double error = Math.hypot(robotPose[0] - points1[3][0], robotPose[1] - points1[3][1]);
            double power1 = 0.0;
            if(error > 2 * SPLINE_ERROR)
                power1 = Range.clip(
                    SPLINE_P * Math.hypot(robotPose[0] - points1[3][0], robotPose[1] - points1[3][1]), 0.0, 1.0);
            else
                break;

            boolean lowGear;

            double turn = 0;
            boolean autoAlign = true;
            if (spline1.desiredT() > 0.6) {
                lowGear = true;
                drivetrain.setDesiredHeading(-180);
                arm.armGoToPosition(1220);

            }else{
                lowGear = false;
            }


            drivetrain.drive(power1 * Math.cos(spline1.angle()), power1 * Math.sin(spline1.angle()), turn, autoAlign, lowGear);
            if (sampleTime.time() > 30) {
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

        waitTime.reset();
        while (waitTime.time() < 1) {
            arm.armGoToPosition(1220);

            drivetrain.drive(0, 0, 0, false, false);

            hand.leftGrabber(true);
            hand.rightGrabber(true);
        }

        hasSeenAprilTag = false;
        visionTimer.reset();

        while (!hasSeenAprilTag && visionTimer.time() <= 1.0) {
            arm.armGoToPosition(1220);

            double[] visionPose = vision.update();
            if (visionPose != null) {
                drivetrain.setPose(visionPose);
                hasSeenAprilTag = true;

                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                arm.armGoToPosition(1220);

            }
        }

        visionTimer.reset();
        while(visionTimer.time() < 2){


        }
    }
}