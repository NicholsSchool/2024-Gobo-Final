package org.firstinspires.ftc.teamcode.subsystems;

//TODO: decide if small april tags are worth it
//TODO: decide if back camera is worth it
//TODO: verify that it works everywhere

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.other.AngleMath;
import org.firstinspires.ftc.teamcode.other.CoordinateMotionProfile;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * The Vision Subsystem of the Robot
 */
public class Vision {
    private final AprilTagProcessor processor;
    private ArrayList<AprilTagDetection> detections;
    private final CoordinateMotionProfile smoothing;
    private double weightsSum;

    /**
     * Instantiates the Vision Subsystem
     *
     * @param hwMap the hardware map
     * @param initialX the starting X
     * @param initialY the starting Y
     */
    public Vision(HardwareMap hwMap, double initialX, double initialY) {
        processor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(processor)
                .build();

        detections = new ArrayList<>();

        smoothing = new CoordinateMotionProfile(initialX, initialY, -72.0, 72.0, 60.0);
    }

    /**
     * Updates the Robot Vision, call in each loop
     *
     * @return the robot pose [x, y, theta] in inches and degrees
     */
    public double[] update() {
        detections = processor.getDetections();
        weightsSum = 0;

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

        double[] smoothedXY = smoothing.update(poseSum[0] / weightsSum, poseSum[1] / weightsSum);

        return new double[]{smoothedXY[0], smoothedXY[1],
                Math.toDegrees(Math.atan2(poseSum[3],poseSum[2]))};
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
        weightsSum += weighting;

        return new double[]{
                weighting * data[0],
                weighting * data[1],
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