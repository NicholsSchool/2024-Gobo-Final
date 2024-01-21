package org.firstinspires.ftc.teamcode.other;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/*
 * Prop Detection Class
 */
public class PropDetector {

    private static final String[] LABELS = {"redFace", "blueFace"};
    private static String TFOD_MODEL_ASSET;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private final HardwareMap hwMap;

    public PropDetector(HardwareMap hwMap) {

        this.hwMap = hwMap;

        TFOD_MODEL_ASSET = "uniPropV1.tflite";

        initTfod();
    }

    public void stopDetecting() {
        visionPortal.close();
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));

        builder.enableLiveView(true);

        builder.addProcessor(tfod);

        visionPortal = builder.build();
    }

    public List<Recognition> getRecognitions() {
        return tfod.getRecognitions();
    }

    public Recognition getBestRecognitions() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        float bestRecConf = 0;

        List<Recognition> recsList = new ArrayList<Recognition>();

        for (Recognition rec : currentRecognitions) {
            float confidence = rec.getConfidence();
            if (confidence > bestRecConf) {
                recsList.add(rec);
                bestRecConf = confidence;
            }
        }

        try {
            return recsList.get(recsList.size() - 1);
        } catch (Exception e) {
            return null;
        }

    }

}