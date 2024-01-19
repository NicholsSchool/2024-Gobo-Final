package org.firstinspires.ftc.teamcode.constants;

/**
 * Arm Constants
 */
public interface ArmConstants {
    /** Shoulder Max Power */
    double SHOULDER_MAX = 0.3;

    /** Climbing Max Power */
    double CLIMB_MAX = 0.75;

    /** Wrist Max Power */
    double WRIST_MAX = 0.5;

    /** Plane Launcher Minimum */
    double PLANE_MIN = 0.35;

    /** Plane Launcher Maximum */
    double PLANE_MAX = 1.0;

    /** Shoulder Proportional Constant */
    double SHOULDER_P = 0.001;

    /** Shoulder Scaling Factor Constant */
    double SHOULDER_F = 0.05;

    /** Arm Starting Position Offset */
    int ARM_STARTING_POS = 550;

    /** Arm Vertical Encoder Position */
    double ARM_VERTICAL = 2770.0;
}
