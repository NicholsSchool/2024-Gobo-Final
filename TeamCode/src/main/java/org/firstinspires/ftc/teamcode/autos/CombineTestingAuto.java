package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.other.Spline;

import org.firstinspires.ftc.teamcode.subsystems.*;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name="Combine Testing")
public class CombineTestingAuto extends LinearOpMode {

    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {
        ElapsedTime sampleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime actionTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        Drivetrain drivetrain = new Drivetrain(hardwareMap, true, 36, -60, 90);
        Arm arm = new Arm(hardwareMap);
        Hand hand = new Hand(hardwareMap);

        double[][] points1 = new double[][]{{36, -60,}, {47.9, 31.4}, {-38.1, 8}, {-38, -58}};
        double[][] points2 = new double[][]{{-38, -58,}, {85.3, -2.6}, {0.7, 25.7}, {-21.8, -6.5}};

        Spline spline1 = new Spline(points1, 20, drivetrain, 100);
        Spline spline2 = new Spline(points2, 20, drivetrain, 100);
        spline1.update();
        spline2.update();

        hand.rightGrabber(true);
        hand.leftGrabber(true);
        waitForStart();

        while(spline1.desiredT() < 0.95){
            spline1.update();
            spline2.update();
            double power1 = 0.7 * Range.clip(1.4 - spline1.desiredT(), 0, 1);

            arm.armGoToPosition(700);
            arm.wristGoToPos(152);

            hand.leftGrabber(true);
            hand.rightGrabber(true);

            drivetrain.drive(power1 * Math.cos(spline1.angle()), power1 * Math.sin(spline1.angle()), 0, true, false);
            drivetrain.setDesiredHeading(0);

            if(sampleTime.time() > 30) {
                spline1.update();
                sampleTime.reset();
            }
            telemetry.addData("x", drivetrain.getXY()[0]);
            telemetry.addData("y", drivetrain.getXY()[1]);
            telemetry.addData("t", spline1.desiredT());
            telemetry.addData("bezierX", spline1.bezierX(spline1.desiredT()));
            telemetry.addData("bezierY", spline1.bezierY(spline1.desiredT()));
            telemetry.addData("t2",(spline2.desiredT()));
            telemetry.addData("arm", arm.getArmPosition());
            telemetry.update();
        }
        actionTime.reset();
        while(actionTime.time() < 2){
            arm.armGoToPosition(-200);
            arm.wristGoToPos(152);

            hand.leftGrabber(true);
            hand.rightGrabber(true);
        }

        hand.leftGrabber(false);
        hand.rightGrabber(false);

        spline2.update();
        while(spline2.desiredT() < 0.95){
            spline2.update();
            double power2 = 0.7 * Range.clip(1.4 - spline2.desiredT(), 0, 1);

            arm.wristGoToPos(152);
            drivetrain.setDesiredHeading(0);
            drivetrain.drive(power2 * Math.cos(spline2.angle()), power2 * Math.sin(spline2.angle()), 0, true, false);
            if(sampleTime.time() > 30) {
                spline2.update();
                sampleTime.reset();
            }
            telemetry.addData("x", drivetrain.getXY()[0]);
            telemetry.addData("y", drivetrain.getXY()[1]);
            telemetry.addData("t", spline2.desiredT());
            telemetry.addData("bezierX", spline2.bezierX(spline2.desiredT()));
            telemetry.addData("bezierY", spline2.bezierY(spline2.desiredT()));
            telemetry.addData("arm", arm.getArmPosition());
            telemetry.update();
        }
        drivetrain.drive(0,0,0,false,false);
    }
}