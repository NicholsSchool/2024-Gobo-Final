package org.firstinspires.ftc.teamcode.other;

/**
 * Robot Constants
 */
public final class Constants {
    /** Denotes Red Alliance in Subsystem Constructors */
    public static final boolean IS_RED_ALLIANCE = false;

    /** Denotes Blue Alliance in Subsystem Constructors */
    public static final boolean IS_BLUE_ALLIANCE = true;


    /**
     * Controller Constants
     */
    public static final class ControllerConstants {
        /** Axis Deadband Value */
        public static final double AXIS_DEADBAND = 0.01;

        /** Waiting time for value being zero long enough */
        public static final double ZERO_WAIT = 0.25;
    }


    /**
     * Motion Profiling Constants
     */
    public static final class ProfileConstants {
        /** Maximum Motion Profile Value */
        public static final double MAX = 0.25;

        /** Maximum Motion Profile Speed */
        public static final double MAX_SPEED = 1.0;

        /** Maximum Coordinate Motion Profile Value */
        public static final double COORDINATE_MAX = 1.0;

        /** Maximum Coordinate Motion Profile Speed */
        public static final double COORDINATE_MAX_SPEED = 2.5;

        /** Maximum Field Motion Profile Value */
        public static final double FIELD_MAX = 72.0;

        /** Maximum Field Motion Profile Speed */
        public static final double FIELD_MAX_SPEED = 60.0;
    }


    /**
     * Arm Constants
     */
    public static final class ArmConstants {
        /** Shoulder Max Power */
        public static final double SHOULDER_MAX = 0.3;

        /** Wrist Max Power */
        public static final double WRIST_MAX = 0.5;

        /** Plane Launcher Minimum */
        public static final double PLANE_MIN = 0.35;

        /** Plane Launcher Maximum */
        public static final double PLANE_MAX = 1.0;

        /** Shoulder Proportional Constant */
        public static final double SHOULDER_P = 0.001;

        /** Shoulder Scaling Factor Constant */
        public static final double SHOULDER_F = 0.05;

        /** Arm Vertical Encoder Position */
        public static final int ARM_VERTICAL = 2850;
    }


    /**
     * Drivetrain Constants
     */
    public static final class DriveConstants {
        /** The Closest to Driver Scoring Y For Blue Alliance */
        public static final double BLUE_SCORING_Y_CLOSE = -42.0;

        /** The Middle Distance to Driver Scoring Y For Blue Alliance */
        public static final double BLUE_SCORING_Y_MID = -36.0;

        /** The Farthest to Driver Scoring Y For Blue Alliance */
        public static final double BLUE_SCORING_Y_FAR = -30.0;

        /** The Farthest to Driver Scoring Y For Red Alliance */
        public static final double RED_SCORING_Y_FAR = 30.0;

        /** The Middle Distance to Driver Scoring Y For Red Alliance */
        public static final double RED_SCORING_Y_MID = 36.0;

        /** The Closest to Driver Scoring Y For Red Alliance */
        public static final double RED_SCORING_Y_CLOSE = 42.0;

        /** Max Motor Speed in Ticks per second */
        public static final int MAX_MOTOR_VEL = 2800;

        /** Splining Proportional Constant */
        public static final double SPLINE_P = 0.025;

        /** Splining allowed error */
        public static final double SPLINE_ERROR = 2.0;

        /** Spline Left Waypoint Coordinate X Value */
        public static final double LEFT_WAYPOINT_X = -14.0;

        /** Spline Right Waypoint Coordinate X Value */
        public static final double RIGHT_WAYPOINT_X = 38.0;

        /** Left Drive Wheel Angle Offset */
        public static final double LEFT_DRIVE_OFFSET = 30.0;

        /** Right Drive Wheel Angle Offset */
        public static final double RIGHT_DRIVE_OFFSET = 150.0;

        /** Back Drive Wheel Angle Offset */
        public static final double BACK_DRIVE_OFFSET = 270.0;

        /** Low Gear Max Speed */
        public static final double LOW_GEAR = 0.45;

        /** High Gear Max Speed */
        public static final double HIGH_GEAR = 0.9;

        /** Auto Align Proportional Constant */
        public static final double AUTO_ALIGN_P = 0.005;

        /** Auto Align allowed error */
        public static final double AUTO_ALIGN_ERROR = 0.5;

        /** Spline Intake Coordinate X Value */
        public static final double INTAKE_X = 56.0;

        /** Spline Intake Coordinate Blue Y Value */
        public static final double BLUE_INTAKE_Y = 56.0;

        /** Spline Intake Coordinate Red Y Value */
        public static final double RED_INTAKE_Y = -56.0;

        /** Spline Waypoint Coordinate Blue Y Value */
        public static final double BLUE_WAYPOINT_Y = 36.0;

        /** Spline Waypoint Coordinate Red Y Value */
        public static final double RED_WAYPOINT_Y = -36.0;

        /** Spline Scoring Coordinate X Value */
        public static final double SCORING_X = -42.0;

        /** Spline Waypoint Coordinate Y Value when going to scoring */
        public static final double WAYPOINT_Y_TO_SCORING = 0.0;

        /** Thru Bore Encoder ticks per revolution */
        public static final int TICKS_PER_REV = 8192;

        /** Dead wheel diameter */
        public static final double DEAD_WHEEL_DIAMETER = 2.5;

        /** Inches per tick of a dead wheel */
        public static final double INCHES_PER_TICK = DEAD_WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;

        /** Horizontal Correction coefficient */
        public static final double STRAFE_ODOMETRY_CORRECTION = 1.0;

        /** Forward Correction coefficient */
        public static final double FORWARD_ODOMETRY_CORRECTION = 1.0;
    }


    /**
     * Hand Constants
     */
    public static final class HandConstants {
        /** Left Grabber In Position */
        public static final double LEFT_IN = 0.65;

        /** Left Grabber Out Position */
        public static final double LEFT_OUT = 0.825;

        public static final double RIGHT_IN = 0.35;

        /** Left Grabber Out Position */
        public static final double RIGHT_OUT = 0.175;
    }


    /**
     * Vision Constants
     */
    public static final class VisionConstants {
        /** Camera Forward Distance */
        public static final double FORWARD_DIST = 5.5;

        /** Camera Horizontal Distance */
        public static final double HORIZONTAL_DIST = 3.0;

        /** Big April Tag Area */
        public static final double BIG_TAG = 25.0;

        /** Small April Tag Area */
        public static final double SMALL_TAG = 4.0;
    }
}