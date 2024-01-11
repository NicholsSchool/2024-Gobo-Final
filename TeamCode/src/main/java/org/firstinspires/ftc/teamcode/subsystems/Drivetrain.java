package org.firstinspires.ftc.teamcode.subsystems;


import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.other.AngleMath;
import org.firstinspires.ftc.teamcode.other.CoordinateMotionProfile;
import org.firstinspires.ftc.teamcode.other.MotionProfile;

//TODO: tune the odometry correction
//TODO: tune the drive motors ff
//TODO: tune the motion profile constants
//TODO: tune auto align and spline constants
//TODO: edit scoring and intaking waypoints
//TODO: change scoring path to go through the center truss

/**
 * Robot Drivetrain Subsystem
 */
public class Drivetrain {
    public static final double BLUE_SCORING_Y_CLOSE = -42.0;
    public static final double BLUE_SCORING_Y_MID = -36.0;
    public static final double BLUE_SCORING_Y_FAR = -30.0;
    public static final double RED_SCORING_Y_FAR = 30.0;
    public static final double RED_SCORING_Y_MID = 36.0;
    public static final double RED_SCORING_Y_CLOSE = 42.0;

    private final int MAX_MOTOR_VEL = 2800;
    private final double SPLINE_P = 0.025;
    private final double SPLINE_ERROR = 2.0;
    private final double LEFT_WAYPOINT_X = -14.0;
    private final double RIGHT_WAYPOINT_X = 38.0;
    private final double BLUE_WAYPOINT_Y = 36.0;
    private final double RED_WAYPOINT_Y = -36.0;

    private final boolean isBlueAlliance;
    private final DcMotorEx leftDrive, rightDrive, backDrive, frontOdometry, leftOdometry, rightOdometry;
    private final AHRS navx;
    private final CoordinateMotionProfile driveProfile;
    private final MotionProfile turnProfile;
    private int previousLeftPosition, previousRightPosition, previousFrontPosition;
    private double x, y, heading, previousHeading, imuOffset, desiredHeading;

    /**
     * Initializes the Drivetrain subsystem
     *
     * @param hwMap the hardwareMap
     * @param isBlueAlliance true for blue, false for red
     * @param x the initial x coordinate
     * @param y the initial y coordinate
     * @param imuOffset the initial value for the robot heading
     */
    public Drivetrain(HardwareMap hwMap, boolean isBlueAlliance, double x, double y, double imuOffset) {
        this.isBlueAlliance = isBlueAlliance;
        this.x = x;
        this.y = y;
        this.heading = imuOffset;
        this.previousHeading = imuOffset;
        this.imuOffset = imuOffset;
        this.desiredHeading = imuOffset;

        leftDrive = hwMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hwMap.get(DcMotorEx.class, "rightDrive");
        backDrive = hwMap.get(DcMotorEx.class, "backDrive");
        leftOdometry = hwMap.get(DcMotorEx.class, "leftOdometry");
        rightOdometry = hwMap.get(DcMotorEx.class, "rightOdometry");
        frontOdometry = hwMap.get(DcMotorEx.class, "rightShoulder");

        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftOdometry.setDirection(DcMotorEx.Direction.FORWARD);
        rightOdometry.setDirection(DcMotorEx.Direction.REVERSE);
        frontOdometry.setDirection(DcMotorEx.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometry.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontOdometry.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftOdometry.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometry.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontOdometry.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        navx = AHRS.getInstance(hwMap.get(NavxMicroNavigationSensor.class,
                "navx"), AHRS.DeviceDataType.kProcessedData);

        driveProfile = new CoordinateMotionProfile();
        turnProfile = new MotionProfile();
    }

    /**
     * A testing and tuning method, spins motors at equal power
     *
     * @param power the power proportion to spin motors  [-1, 1]
     */
    public void simpleSpin(double power) {
        leftDrive.setVelocity(power * MAX_MOTOR_VEL);
        rightDrive.setVelocity(power * MAX_MOTOR_VEL);
        backDrive.setVelocity(power * MAX_MOTOR_VEL);
    }

    /**
     * Sets the Drive Wheels to Float Zero Power Behavior
     */
    public void setFloat() {
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Drives the robot field oriented
     *
     * @param power the driving power proportion
     * @param angle the angle to drive at in degrees
     * @param turn the turning power proportion
     * @param autoAlign whether to autoAlign
     */
    public void drive(double power, double angle, double turn, boolean autoAlign) {
        if(autoAlign)
            turn = turnProfile.update(turnToAngle());
        else
            turn = turnProfile.update(Range.clip(turn, -0.25, 0.25));

        power = Range.clip(power, 0.0, 1.0 - turn);
        double angleRadians = Math.toRadians(angle);
        double[] xy = driveProfile.update(power * Math.cos(angleRadians), power * Math.sin(angleRadians));
        power = Math.hypot(xy[0], xy[1]);
        angle = Math.toDegrees(Math.atan2(xy[1], xy[0]));

        leftDrive.setVelocity((turn + power * Math.cos(Math.toRadians(angle + 30.0 - heading))) * MAX_MOTOR_VEL);
        rightDrive.setVelocity((turn + power * Math.cos(Math.toRadians(angle + 150.0 - heading))) * MAX_MOTOR_VEL);
        backDrive.setVelocity((turn + power * Math.cos((Math.toRadians(angle + 270.0 - heading)))) * MAX_MOTOR_VEL);
    }

    /**
     * Spins the robot anchor-less to the desired heading
     *
     * @return the turning speed as a proportion
     */
    public double turnToAngle() {
        double error = AngleMath.addAngles(heading, -desiredHeading);
        if(Math.abs(error) < 0.5)
            return 0.0;
        return Range.clip(error * 0.005, -0.25, 0.25);
    }

    /**
     * Sets the heading to auto-align to
     *
     * @param desiredHeading the heading in degrees
     */
    public void setDesiredHeading(double desiredHeading) {
        this.desiredHeading = desiredHeading;
    }

    /**
     * With the robot at (x, y), calculates the drive angle of the robot
     * in order to follow a parabola and arrive at the waypoint (wx, wy)
     * that is the parabola's vertex.
     * The parabola is defined to contain the robot's coordinates.
     *
     * @param wx the waypoint x coordinate
     * @param wy the waypoint y coordinate
     * @param toIntake whether the robot is going to the intake
     *
     * @return the drive angle in degrees [-180, 180)
     */
    public double angleToVertex(double wx, double wy, boolean toIntake) {
        if(x == wx)
            return toIntake ? 0.0 : -180.0;
        double offset = x > wx ? -180.0 : 0.0;
        return Math.toDegrees(Math.atan(2.0 * (y - wy) / (x - wx))) + offset;
    }

    /**
     * With the robot at (x, y), calculates the drive angle of the robot
     * in order to follow a parabola and arrive at the waypoint (wx, wy).
     * The parabola is defined with its vertex constrained to the x-value
     * of h (the previous waypoint), and the curve consists of both the
     * waypoint and robot coordinates.
     *
     * @param wx the waypoint x coordinate
     * @param wy the waypoint y coordinate
     * @param h  the x value of the previous waypoint
     * @param toIntake whether the robot is going to the intake
     * @return the drive angle in degrees [-180, 180)
     */
    public double angleFromVertex(double wx, double wy, double h, boolean toIntake) {
        if(x == h)
            return toIntake ? 0.0 : -180.0;

        double robotDiff = Math.pow(x - h, 2);
        double waypointDiff = Math.pow(wx - h, 2);

        if(robotDiff == waypointDiff)
            return y > wy ? -90.0 : 90.0;

        double k = (wy * robotDiff - y * waypointDiff) / (robotDiff - waypointDiff);
        double offset = x > wx ? -180.0 : 0.0;
        return Math.toDegrees(Math.atan(2.0 * (y - k) / (x - h))) + offset;
    }

    /**
     * Automatically directs the robot to the Coordinates of the Correct Intake
     * area using parabolas in piecewise.
     */
    public void splineToIntake(double turn, boolean autoAlign) {
        double INTAKE_X = 56.0;
        double BLUE_INTAKE_Y = 56.0;
        double RED_INTAKE_Y = -56.0;

        double distance = Math.hypot(INTAKE_X - x, isBlueAlliance ? BLUE_INTAKE_Y - y : RED_INTAKE_Y - y);
        double power = distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0;

        double angle;
        if(x < LEFT_WAYPOINT_X)
            angle = angleToVertex(LEFT_WAYPOINT_X, isBlueAlliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, true);
        else if(x < RIGHT_WAYPOINT_X)
            angle = angleToVertex(RIGHT_WAYPOINT_X, isBlueAlliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, true);
        else
            angle = angleFromVertex(INTAKE_X, isBlueAlliance ? BLUE_INTAKE_Y : RED_INTAKE_Y, RIGHT_WAYPOINT_X, true);

        drive(power, angle, turn, autoAlign);
    }

    /**
     * Automatically directs the robot to the Coordinates of the Correct Backstage
     * area using parabolas in piecewise.
     */
    public void splineToScoring(double turn, boolean autoAlign, double scoringY) {
        double SCORING_X = -42.0;

        double distance = Math.hypot(SCORING_X - x, scoringY - y);
        double power = distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0;

        double angle;
        if(x > RIGHT_WAYPOINT_X)
            angle = angleToVertex(RIGHT_WAYPOINT_X, isBlueAlliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, false);
        else if(x > LEFT_WAYPOINT_X)
            angle = angleToVertex(LEFT_WAYPOINT_X, isBlueAlliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, false);
        else
            angle = angleFromVertex(SCORING_X, scoringY, LEFT_WAYPOINT_X, false);

        drive(power, angle, turn, autoAlign);
    }

    /**
     * Updates Pose using Odometry Wheels
     */
    public void update() {
        final int TICKS_PER_REV = 8192;
        final double DEAD_DIAMETER = 2.5;
        final double INCHES_PER_TICK = DEAD_DIAMETER * Math.PI / TICKS_PER_REV;
        final double STRAFE_ODOMETRY_CORRECTION = 1.0;
        final double FORWARD_ODOMETRY_CORRECTION = 1.0;

        int currentLeft = leftOdometry.getCurrentPosition();
        int currentRight = rightOdometry.getCurrentPosition();
        int currentFront = frontOdometry.getCurrentPosition();

        int deltaLeft = currentLeft - previousLeftPosition;
        int deltaRight = currentRight - previousRightPosition;
        int deltaFront = currentFront - previousFrontPosition;

        double deltaX = (deltaLeft + deltaRight) * INCHES_PER_TICK * STRAFE_ODOMETRY_CORRECTION;
        double deltaY = deltaFront * INCHES_PER_TICK * FORWARD_ODOMETRY_CORRECTION;

        heading = -navx.getYaw() + imuOffset;

        double adjustedHeadingRadians = Math.toRadians((heading + previousHeading) * 0.5);

        x += deltaX * Math.sin(adjustedHeadingRadians) + deltaY * Math.cos(adjustedHeadingRadians);
        y += -deltaX * Math.cos(adjustedHeadingRadians) + deltaY * Math.sin(adjustedHeadingRadians);

        previousLeftPosition = currentLeft;
        previousRightPosition = currentRight;
        previousFrontPosition = currentFront;
        previousHeading = heading;

    }

    /**
     * Sets the Robot Pose using the IMU offset
     *
     * @param pose the x, y, theta of the robot
     */
    public void setPose(double[] pose) {
        x = pose[0];
        y = pose[1];
        imuOffset = pose[2] + navx.getYaw();
    }

    /**
     * Get Motor Velocities for telemetry
     *
     * @return respectively: left, right back velocities
     */
    public double[] getMotorVelocities() {
        return new double[]{
                leftDrive.getVelocity(),
                rightDrive.getVelocity(),
                backDrive.getVelocity(),
        };
    }

    /**
     * Get the dead wheel position values for telemetry
     *
     * @return the dead wheel encoder values in the order:
     * left, right, front positions
     */
    public double[] getOdometryPositions() {
        return new double[]{
                leftOdometry.getCurrentPosition(),
                rightOdometry.getCurrentPosition(),
                frontOdometry.getCurrentPosition()
        };
    }

    /**
     * Get the robot coordinates for telemetry
     *
     * @return the x and y value of the center of the bot
     */
    public double[] getXY() {
        return new double[]{x, y};
    }

    /**
     * Gets the heading of the robot on the field coordinate system
     *
     * @return the heading in degrees
     */
    public double getFieldHeading() {
        return AngleMath.addAngles(heading, 0.0);
    }
}