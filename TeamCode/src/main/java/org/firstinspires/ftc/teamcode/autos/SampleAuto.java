package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.other.Spline;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name="SPLINE MAYBE")
public class SampleAuto extends LinearOpMode {

    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {
        ElapsedTime sampleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, true, 36, -60, 90);

        double[][] points = new double[][]{{36, -60,}, {47.9, 31.4}, {-38.1, 8}, {-38, 58}};

        Spline spline = new Spline(points, 20, drivetrain, 100);
        spline.update();
        waitForStart();

        while(spline.desiredT() < 0.9){
            spline.update();
            drivetrain.drive(0.7 * Math.cos(spline.angle()), 0.7 * Math.sin(spline.angle()), 0, false, false);
            if(sampleTime.time() > 30) {
                spline.update();
                sampleTime.reset();
            }
            telemetry.addData("x", drivetrain.getXY()[0]);
            telemetry.addData("y", drivetrain.getXY()[1]);
            telemetry.addData("t", spline.desiredT());
            telemetry.update();
        }
        drivetrain.drive(0,0,0,false,false);
    }
}