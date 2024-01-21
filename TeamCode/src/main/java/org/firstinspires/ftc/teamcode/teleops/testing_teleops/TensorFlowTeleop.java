package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.other.PropDetector;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TensorFlow Testing")
public class TensorFlowTeleop extends OpMode {
    PropDetector pd;

    public void init() {
        pd = new PropDetector(hardwareMap);
    }

    public void loop() {
        Recognition bestRec = pd.getARandomRecognition();

        if (bestRec == null)
            telemetry.addData("no recognitions", "");

        telemetry.addData("> PREDICTION CONFIDENCE", bestRec.getConfidence());
        telemetry.addData("> LABEL", bestRec.getLabel());
        telemetry.addData("LEFT CORNER", bestRec.getLeft());

    }

    public void stop() {

        pd.stopDetecting();

    }

}