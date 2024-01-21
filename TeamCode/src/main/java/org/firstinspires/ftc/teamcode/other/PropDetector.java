package org.firstinspires.ftc.teamcode.other;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/*
 * Prop Detection Class
 */
public class PropDetector {

    private static final String[] LABELS = {"redFace", "blueFace"};
    private final TfodProcessor tfod;
    private final VisionPortal visionPortal;

    public PropDetector(HardwareMap hwMap) {

        String TFOD_MODEL_ASSET = "uniPropV1.tflite";

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.enableLiveView(true);
        builder.addProcessor(tfod);

        visionPortal = builder.build();
    }

    public void stopDetecting() {
        visionPortal.close();
    }

    public List<Recognition> getRecognitions() {
        return tfod.getRecognitions();
    }

    public Recognition getARandomRecognition() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        float bestRecConf = 0;


        for (Recognition rec : currentRecognitions) {
            return rec;
        }
        return null;
    }
}