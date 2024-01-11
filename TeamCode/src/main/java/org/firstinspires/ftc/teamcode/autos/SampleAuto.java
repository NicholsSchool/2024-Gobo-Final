package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name="SAMPLE AUTO")
public class SampleAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {
        waitForStart();
        runtime.reset();
    }
}