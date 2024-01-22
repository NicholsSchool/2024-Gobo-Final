package org.firstinspires.ftc.teamcode.constants;

/**
 * Drivetrain Constants
 */
public interface DriveConstants {
    /** The Closest to Driver Scoring Y For Blue Alliance */
    double BLUE_SCORING_Y_CLOSE = -42.0;

    /** The Middle Distance to Driver Scoring Y For Blue Alliance */
    double BLUE_SCORING_Y_MID = -36.0;

    /** The Farthest to Driver Scoring Y For Blue Alliance */
    double BLUE_SCORING_Y_FAR = -30.0;

    /** The Farthest to Driver Scoring Y For Red Alliance */
    double RED_SCORING_Y_FAR = 30.0;

    /** The Middle Distance to Driver Scoring Y For Red Alliance */
    double RED_SCORING_Y_MID = 36.0;

    /** The Closest to Driver Scoring Y For Red Alliance */
    double RED_SCORING_Y_CLOSE = 42.0;

    /** Auto Driving Proportional Constant */
    double SPLINE_P = 0.05;

    /** Auto Driving allowed error */
    double SPLINE_ERROR = 2.0;

    /** Spline Left Waypoint Coordinate X Value */
    double LEFT_WAYPOINT_X = -14.0;

    /** Spline Right Waypoint Coordinate X Value */
    double RIGHT_WAYPOINT_X = 38.0;

    /** Left Drive Wheel Angle Offset */
    double LEFT_DRIVE_OFFSET = Math.PI / 6.0;

    /** Right Drive Wheel Angle Offset */
    double RIGHT_DRIVE_OFFSET = 5.0 * Math.PI / 6.0;

    /** Back Drive Wheel Angle Offset */
    double BACK_DRIVE_OFFSET = 3.0 * Math.PI / 2.0;

    /** Low Gear Max Speed */
    double LOW_GEAR = 0.375;

    /** High Gear Max Speed */
    double HIGH_GEAR = 0.75;

    /** Auto Align Proportional Constant */
    double AUTO_ALIGN_P = 0.005;

    /** Auto Align allowed error */
    double AUTO_ALIGN_ERROR = 0.5;

    /** Spline Intake Coordinate X Value */
    double INTAKE_X = 56.0;

    /** Spline Intake Coordinate Blue Y Value */
    double BLUE_INTAKE_Y = 56.0;

    /** Spline Intake Coordinate Red Y Value */
    double RED_INTAKE_Y = -56.0;

    /** Spline Waypoint Coordinate Blue Y Value */
    double BLUE_WAYPOINT_Y = 36.0;

    /** Spline Waypoint Coordinate Red Y Value */
    double RED_WAYPOINT_Y = -36.0;

    /** Spline Scoring Coordinate X Value */
    double SCORING_X = -42.0;

    /** Spline Waypoint Coordinate Y Value when going to scoring */
    double WAYPOINT_Y_TO_SCORING = 0.0;

    /** Thru Bore Encoder ticks per revolution */
    int THRU_BORE_TICKS_PER_REV = 8192;

    /** Dead wheel diameter in inches */
    double DEAD_WHEEL_DIAMETER = 2.5;

    /** Inches per tick of a dead wheel */
    double INCHES_PER_TICK = DEAD_WHEEL_DIAMETER * Math.PI / THRU_BORE_TICKS_PER_REV;

    /** Horizontal Correction coefficient */
    double STRAFE_ODOMETRY_CORRECTION = 0.576;

    /** Forward Correction coefficient */
    double FORWARD_ODOMETRY_CORRECTION = 0.964;
}
