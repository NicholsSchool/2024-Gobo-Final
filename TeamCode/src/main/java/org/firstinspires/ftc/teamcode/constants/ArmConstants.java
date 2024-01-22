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

    /** Wrist Proportional Constant */
    double WRIST_P = 0.0025;

    /** Core Hex Ticks per Revolution */
    double CORE_HEX_TICKS_PER_REV = 288;

    /** Wrist Encoder Offset For Start Of Auto*/
    double WRIST_AUTO_POSITION = -192;

    /**
     * Wrist down virtual fourbar position
     */
    double WRIST_DOWN = 5.0;

    /**
     * Wrist up virtual fourbar position
     */
    double WRIST_UP = 48.0;

    /**
     * Arm position to switch the wrist virtual fourbar position
     */
    double WRIST_SWITCHING_POS = 300;
}
