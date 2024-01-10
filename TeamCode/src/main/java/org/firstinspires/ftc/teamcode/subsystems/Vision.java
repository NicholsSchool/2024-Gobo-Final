package org.firstinspires.ftc.teamcode.subsystems;

//TODO: localization smoothing using profile
//TODO: camera exposure tuning
//TODO: decide after all that if small april tags are worth it

//TODO: experiment with loop() times with pause/not calling the method, etc
//TODO: test loop time with MJPEG format
//TODO: separate method for camera 2, test loop times

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.other.AngleMath;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * The Vision Subsystem of the Robot
 */
public class Vision {
    AprilTagProcessor processor;
    VisionPortal portal;
    ArrayList<AprilTagDetection> detections;

    /**
     * Instantiates the Vision Subsystem
     *
     * @param hwMap the hardware map
     */
    public Vision(HardwareMap hwMap) {
        processor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(processor)
                .build();

        detections = new ArrayList<>();
    }

    /**
     * Updates the Robot Vision, call in each loop
     *
     * @return the robot pose [x, y, theta] in inches and degrees
     */
    public double[] update() {
        detections = processor.getDetections();

        double[] poseSum = new double[4];

        int size = detections.size();
        if(size == 0)
            return null;

        for(int i = 0; i < size; i++) {
            double[] currentPose = addWeights(localize(i));
            poseSum[0] += currentPose[0];
            poseSum[1] += currentPose[1];
            poseSum[2] += currentPose[2];
            poseSum[3] += currentPose[3];
        }

        //TODO: figure out how to get the weighted sum to the right number
//        return new double[]{averagedPose[0], averagedPose[1],
//                Math.toDegrees(Math.atan2(poseSum[3],poseSum[2]))};
        return null;
    }

    private double[] localize(int i) {
        final double FORWARD_DIST = 5.5;
        final double HORIZONTAL_DIST = 3.0;

        AprilTagDetection aprilTagDetection = detections.get(i);

        int id = aprilTagDetection.id;
        boolean isScoringTag = id <= 6;

        double range = aprilTagDetection.ftcPose.range;
        double yaw = aprilTagDetection.ftcPose.yaw;
        double bearing = aprilTagDetection.ftcPose.bearing;

        double cameraDeltaX = range * Math.cos(Math.toRadians(bearing - yaw));
        double cameraDeltaY = range * Math.sin(Math.toRadians(bearing - yaw));

        double cameraX = isScoringTag ? cameraDeltaX - 60.25 : 70.25 - cameraDeltaX;
        double cameraY = isScoringTag ?
                getTagYCoordinate(id) + cameraDeltaY : getTagYCoordinate(id) - cameraDeltaY;

        double fieldHeadingInRadians =
                Math.toRadians(isScoringTag ? AngleMath.addAngles(-yaw, -180.0) : -yaw);

        double localizedX = cameraX - HORIZONTAL_DIST * Math.sin(fieldHeadingInRadians)
                - FORWARD_DIST * Math.cos(fieldHeadingInRadians);

        double localizedY = cameraY - FORWARD_DIST * Math.sin(fieldHeadingInRadians)
                + HORIZONTAL_DIST * Math.cos(fieldHeadingInRadians);

        return new double[]{localizedX, localizedY, fieldHeadingInRadians, range, id};
    }

    private double getTagYCoordinate(int id) {
        switch(id) {
            case 1:
                return -41.41;
            case 2:
                return -35.41;
            case 3:
                return -29.41;
            case 4:
                return 29.41;
            case 5:
                return 35.41;
            case 6:
                return 41.41;
            case 7:
                return 40.625;
            case 8:
                return 35.125;
            case 9:
                return -35.125;
            default:
                return -40.625;
        }
    }

    private double[] addWeights(double[] data) {
        double weighting = (data[4] == 7.0 || data[4] == 10.0 ? 25.0 : 4.0) / (data[3] * data[3]);

        return new double[]{
                data[0] * weighting,
                data[1] * weighting,
                weighting * Math.cos(data[2]),
                weighting * Math.sin(data[2])
        };
    }

    /**
     * Returns the number of April Tag Detections
     *
     * @return the number of detections
     */
    public int getNumDetections() {
        return detections.size();
    }
}