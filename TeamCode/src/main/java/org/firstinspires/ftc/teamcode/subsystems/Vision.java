package org.firstinspires.ftc.teamcode.subsystems;

//TODO: redo and check all localization math and weighted average based on size and distance
//TODO: localization smoothing using profile
//TODO: camera exposure tuning
//TODO: decide after all that if small april tags are worth it
//TODO: see if april tags tell you their position from their metadata

//TODO: experiment with loop() times with pause/not calling the method, etc
//TODO: test loop time with MJPEG format
//TODO: separate method for camera 2, test loop times

//TODO: put angle related math in the AngleMath class

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
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();

        portal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2).addProcessor(processor).build();

        detections = new ArrayList<>();
    }

//    /**
//     * Updates the Robot Vision, call in each loop
//     *
//     * @return the robot pose [x, y, theta] in inches and degrees
//     */
//    public double[] update() {
//        detections = processor.getDetections();
//
//        double[] averagedPose = new double[4];
//
//        int frontSize = frontDetections.size();
//        for(int i = 0; i < frontSize; i++) {
//            double[] pose = localize(i, true);
//            averagedPose[0] += pose[0];
//            averagedPose[1] += pose[1];
//            averagedPose[2] += pose[3] == 1 ? 5 * Math.cos( Math.toRadians(pose[2]) ) : Math.cos( Math.toRadians(pose[2]) );
//            averagedPose[3] += pose[3] == 1 ? 5 * Math.sin( Math.toRadians(pose[2]) ) : Math.sin( Math.toRadians(pose[2]) );
//        }
//
//        int backSize = backDetections.size();
//        for(int i = 0; i < backSize; i++) {
//            double[] pose = localize(i, false);
//            averagedPose[0] += pose[0];
//            averagedPose[1] += pose[1];
//            averagedPose[2] += pose[3] == 1.0 ? 5 * Math.cos( Math.toRadians(pose[2]) ) : Math.cos( Math.toRadians(pose[2]) );
//            averagedPose[3] += pose[3] == 1.0 ? 5 * Math.sin( Math.toRadians(pose[2]) ) : Math.sin( Math.toRadians(pose[2]) );
//        }
//
//        if(frontSize + backSize == 0)
//            return null;
//
//        averagedPose[0] /= (frontSize + backSize);
//        averagedPose[1] /= (frontSize + backSize);
//
//        double heading = MathUtilities.addAngles(Math.toDegrees( Math.atan2(averagedPose[3],averagedPose[2]) ), 0.0);
//        return new double[]{averagedPose[0], averagedPose[1], heading};
//    }

    private double[] localize(int i, boolean isFrontCam) {
        final double FRONT_CAM_FORWARD_DIST = 5.5; //TODO: change this
        final double FRONT_CAM_HORIZONTAL_DIST = 3.125;

        AprilTagDetection aprilTagDetection = detections.get(i);

        int id = aprilTagDetection.id;
        double range = aprilTagDetection.ftcPose.range;
        double yaw = aprilTagDetection.ftcPose.yaw;
        double bearing = aprilTagDetection.ftcPose.bearing;
        boolean isScoringTag = id <= 6;

        double tagX = isScoringTag ? -61.5 : 72.125;
        double tagY = getTagYCoordinate(id);

        double fieldHeading = isScoringTag ? AngleMath.addAngles(-yaw, -180.0) : -yaw;

        double cameraDeltaX = range * Math.cos(Math.toRadians(bearing - yaw));
        double cameraDeltaY = range * Math.sin(Math.toRadians(bearing - yaw));

        double cameraX = isScoringTag ? tagX + cameraDeltaX : tagX - cameraDeltaX;
        double cameraY = isScoringTag ? tagY + cameraDeltaY : tagY - cameraDeltaY; //TODO: is this right???

        double fieldHeadingInRadians = Math.toRadians(fieldHeading);

        double localizedX = cameraX - FRONT_CAM_FORWARD_DIST * Math.cos(fieldHeadingInRadians)
                - FRONT_CAM_HORIZONTAL_DIST * Math.sin(fieldHeadingInRadians);
        double localizedY = cameraY - FRONT_CAM_HORIZONTAL_DIST * Math.cos(fieldHeadingInRadians)
                - FRONT_CAM_FORWARD_DIST * Math.sin(fieldHeadingInRadians); //TODO: check this too

        //TODO: change this and averaging logic
        return new double[] {localizedX, localizedY, fieldHeading, id == 7 || id == 10 ? 1.0 : 0.0};
    }

    private double getTagYCoordinate(int id) {
        switch(id) {
            case 2:
            case 9:
                return -36.0;
            case 3:
                return -30.0;
            case 4:
                return 30.0;
            case 5:
            case 8:
                return 36.0;
            case 6:
            case 7:
                return 42.0;
            default:
                return -42.0;
        }
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