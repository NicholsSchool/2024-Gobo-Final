package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.other.PropDetector;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TeleopToTestTheTensorflowTemporalTrackingTelemetryTheoryToTelepathicallyTechnicateTheTelecommunication")
public class VisionTesting extends OpMode {

    PropDetector pd;

    public void init() {

        pd = new PropDetector(hardwareMap, true);

    }

    public void loop() {

        Recognition bestRec = pd.getBestRecognitions();

        if (bestRec == null) { return; }

        telemetry.addLine("THIS AINT WORKIN!!!");
        telemetry.addData("> PREDICTION CONFIDENCE", bestRec.getConfidence());
        telemetry.addData("> LABEL", bestRec.getLabel());
        telemetry.addData("LEFT CORNER", bestRec.getLeft());

    }

    public void stop() {

        pd.stopDetecting();

    }

}