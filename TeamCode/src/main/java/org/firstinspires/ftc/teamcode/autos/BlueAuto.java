package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.other.PropDetector;
import org.firstinspires.ftc.teamcode.other.Spline;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.List;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name = "Blue Auto")
public class BlueAuto extends LinearOpMode implements DriveConstants, ArmConstants, TeleopConstants {
    enum PropZones {
        LEFT,
        CENTER,
        RIGHT
    }

    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {
        ElapsedTime waitTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        ElapsedTime sampleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Drivetrain drivetrain = new Drivetrain(hardwareMap, IS_BLUE_ALLIANCE, 36, -65, 90.0);
        Arm arm = new Arm(hardwareMap);
        Hand hand = new Hand(hardwareMap);
        hand.leftGrabber(true);
        hand.rightGrabber(true);
        PropDetector propDetector = new PropDetector(hardwareMap);
        List<Recognition> recs;

        Lights lights = new Lights(hardwareMap, false);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);


        double[][] points1 = new double[][]{{36.0, -38.0}, {43.0, -57.0}, {55.5, -33.6}, {49.0, -10.8}};
        double[][] points2 = new double[][]{{49.0, -10.8}, {2.6, 0.1}, {-13.5, -8.9}, {-57.3, -5.0}};

        Spline spline1 = new Spline(points1, 20, drivetrain, 100);
        spline1.update();

        Spline spline2 = new Spline(points2, 20, drivetrain, 100);
        spline2.update();

        Recognition bestRec = null;
        int propPosition = 1;

        while(opModeInInit()) {
            bestRec = propDetector.getARandomRecognition();
            telemetry.addData("best rec", bestRec == null ? "" : bestRec);
            telemetry.addData("number", bestRec == null ? "" : (bestRec.getLeft() + bestRec.getRight()) * 0.5);
            telemetry.update();
        }


        waitForStart();


        propDetector.stopDetecting();
        Vision vision = new Vision(hardwareMap, 36.0, -65.0);


        double purplePixelAngle;

        PropZones propZone;

        if(bestRec != null) {
            int point = (int) ((bestRec.getLeft() + bestRec.getRight()) / 2.0);
            if (point < 450.0) {
                propZone = PropZones.CENTER;
                purplePixelAngle = 75.0;
            } else {
                propZone = PropZones.RIGHT;
                purplePixelAngle = 55.0;
            }
        }
        else {
            propZone = PropZones.LEFT;
            purplePixelAngle = 175.0;
        }

        double[] scanCoords = new double[]{36.0, -38.0};
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
        while (waitTime.time() < 1.0) {
            drivetrain.update();
        }

        boolean hasSeenAprilTag = false;
        ElapsedTime visionTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while (!hasSeenAprilTag && visionTimer.time() <= 2.0) {
            double[] visionPose = vision.update();
            if (visionPose != null) {
                drivetrain.setPose(visionPose);
                hasSeenAprilTag = true;
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
            }
        }

        waitTime.reset();
        while (waitTime.time() < 0.25) {
            drivetrain.update();
        }

        drivetrain.setDesiredHeading(purplePixelAngle);
        double error = AUTO_ALIGN_ERROR;
        while (error >= AUTO_ALIGN_ERROR) {
            drivetrain.update();
            double angle = drivetrain.getFieldHeading();
            drivetrain.drive(0.0, 0.0, 0.0, true, true);
            error = Math.abs(angle - purplePixelAngle);
        }
        drivetrain.drive(0, 0, 0, false, false);

        waitTime.reset();
        while (waitTime.time() < 0.25) {
            drivetrain.update();
        }

        waitTime.reset();
        while(waitTime.time() < 2.0) {
            drivetrain.update();
            arm.armGoToPosition(1700.0 - ARM_STARTING_POS);
        }

        waitTime.reset();
        while(waitTime.time() < 2.0) {
            drivetrain.update();
            arm.wristGoToPos(-WRIST_AUTO_POSITION);
        }

        waitTime.reset();
        while(waitTime.time() < 1.0) {
            drivetrain.update();
            arm.armGoToPosition(-ARM_STARTING_POS);
            arm.wristGoToPos(-WRIST_AUTO_POSITION);
        }

        if(propZone.equals(PropZones.RIGHT)) {
            drivetrain.setDesiredHeading(30.0);
            double otherError = AUTO_ALIGN_ERROR;
            while (otherError >= AUTO_ALIGN_ERROR) {
                drivetrain.update();
                double angle = drivetrain.getFieldHeading();
                drivetrain.drive(0.0, 0.0, 0.0, true, true);
                otherError = Math.abs(angle - 30.0);
            }
            drivetrain.drive(0, 0, 0, false, false);
        }

        waitTime.reset();
        while(waitTime.time() < 1.0) {
            drivetrain.update();
            arm.wristGoToPos(-WRIST_AUTO_POSITION);
        }

        waitTime.reset();
        while(waitTime.time() < 1.0) {
            drivetrain.update();
            hand.rightGrabber(false);
        }

        waitTime.reset();
        while(waitTime.time() < 2.0) {
            drivetrain.update();
            arm.wristGoToPos(60.0 - WRIST_AUTO_POSITION);
        }

        hand.rightGrabber(true);

        while(spline1.desiredT() < 0.98) {
            spline1.update();
            double[] robotPose = drivetrain.getXY();

            hand.leftGrabber(true);
            hand.rightGrabber(true);

            double jamesSplineError = Math.hypot(robotPose[0] - points1[3][0], robotPose[1] - points1[3][1]);
            double power1 = 0.0;
            if(jamesSplineError > 2 * SPLINE_ERROR)
                power1 = Range.clip(
                        SPLINE_P * Math.hypot(robotPose[0] - points1[3][0], robotPose[1] - points1[3][1]), 0.0, 1.0);
            else
                break;

            boolean lowGear;

            double turn = 0;
            boolean autoAlign = true;

            double desiredT = spline1.desiredT();

            lowGear = desiredT < 0.7 || desiredT > 0.8;
            drivetrain.setDesiredHeading(0.0);

            drivetrain.drive(power1 * Math.cos(spline1.angle()), power1 * Math.sin(spline1.angle()), turn, autoAlign, lowGear);
            if (sampleTime.time() > 20) {
                spline1.update();
                sampleTime.reset();
            }
            telemetry.addData("x", drivetrain.getXY()[0]);
            telemetry.addData("y", drivetrain.getXY()[1]);
            telemetry.addData("t", spline1.desiredT());
            telemetry.addData("bezierX", spline1.bezierX(spline1.desiredT()));
            telemetry.addData("bezierY", spline1.bezierY(spline1.desiredT()));
            telemetry.update();
        }

        while(spline2.desiredT() < 0.98) {
            spline2.update();
            double[] robotPose = drivetrain.getXY();

            hand.leftGrabber(true);
            hand.rightGrabber(true);

            double jamesSplineError = Math.hypot(robotPose[0] - points2[3][0], robotPose[1] - points2[3][1]);
            double power2 = 0.0;
            if(jamesSplineError > 2 * SPLINE_ERROR)
                power2 = Range.clip(
                        SPLINE_P * Math.hypot(robotPose[0] - points2[3][0], robotPose[1] - points2[3][1]), 0.0, 1.0);
            else
                break;

            boolean lowGear;

            double turn = 0;
            boolean autoAlign = true;

            double desiredT = spline2.desiredT();

            lowGear = desiredT < 0.7 || desiredT > 0.8;
            if(desiredT > 0.77)
                drivetrain.setDesiredHeading(-180.0);
            else
                drivetrain.setDesiredHeading(0.0);

            drivetrain.drive(power2 * Math.cos(spline2.angle()), power2 * Math.sin(spline2.angle()), turn, autoAlign, lowGear);
            if (sampleTime.time() > 20) {
                spline2.update();
                sampleTime.reset();
            }
            telemetry.addData("x", drivetrain.getXY()[0]);
            telemetry.addData("y", drivetrain.getXY()[1]);
            telemetry.addData("t", spline1.desiredT());
            telemetry.addData("bezierX", spline1.bezierX(spline1.desiredT()));
            telemetry.addData("bezierY", spline1.bezierY(spline1.desiredT()));
            telemetry.update();
        }

        drivetrain.drive(0, 0, 0, false, false);
        hand.leftGrabber(false);

        drivetrain.setDesiredHeading(-180.0);
        double anotherError = AUTO_ALIGN_ERROR;
        while (anotherError >= AUTO_ALIGN_ERROR) {
            drivetrain.update();
            double angle = drivetrain.getFieldHeading();
            drivetrain.drive(0.0, 0.0, 0.0, true, true);
            anotherError = Math.abs(angle + 180.0);
        }
        drivetrain.drive(0, 0, 0, false, false);

        terminateOpModeNow();
    }
}