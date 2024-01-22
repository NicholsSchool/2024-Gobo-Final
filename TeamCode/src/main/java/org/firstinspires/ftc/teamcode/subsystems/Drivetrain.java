package org.firstinspires.ftc.teamcode.subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.other.AngleMath;
import org.firstinspires.ftc.teamcode.other.CoordinateMotionProfile;
import org.firstinspires.ftc.teamcode.other.MotionProfile;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;

/**
 * Robot Drivetrain Subsystem
 */
public class Drivetrain implements DriveConstants {
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
     * @param initialHeading the initial robot heading in Radians
     */
    public Drivetrain(HardwareMap hwMap, boolean isBlueAlliance, double x, double y, double initialHeading) {
        this.isBlueAlliance = isBlueAlliance;
        this.x = x;
        this.y = y;
        this.heading = initialHeading;
        this.previousHeading = initialHeading;
        this.imuOffset = initialHeading;
        this.desiredHeading = initialHeading;

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
        leftOdometry.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightOdometry.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontOdometry.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
     * @param power the power proportion to spin motors
     */
    public void simpleSpin(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        backDrive.setPower(power);
    }

    /**
     * Drives the robot field oriented
     *
     * @param xInput the x input value
     * @param yInput the y input value
     * @param turn the turning power proportion
     * @param autoAlign whether to autoAlign
     * @param lowGear whether to put the robot to virtual low gear
     */
    public void drive(double xInput, double yInput, double turn, boolean autoAlign, boolean lowGear) {
        turn = turnProfile.update(autoAlign ? turnToAngle() : turn);

        double[] xy = driveProfile.update(xInput, yInput,(lowGear ? LOW_GEAR : HIGH_GEAR) - Math.abs(turn));
        double power = Math.hypot(xy[0], xy[1]);
        double angle = Math.atan2(xy[1], xy[0]);

        leftDrive.setPower(turn + power * Math.cos(Math.toRadians(angle + LEFT_DRIVE_OFFSET - heading)));
        rightDrive.setPower(turn + power * Math.cos(Math.toRadians(angle + RIGHT_DRIVE_OFFSET - heading)));
        backDrive.setPower(turn + power * Math.cos(Math.toRadians(angle + BACK_DRIVE_OFFSET - heading)));
    }

    private double turnToAngle() {
        double error = AngleMath.addAnglesRadians(heading, -desiredHeading);
        return Math.abs(error) < AUTO_ALIGN_ERROR ? 0.0 : error * AUTO_ALIGN_P;
    }

    /**
     * Sets the heading to auto-align to
     *
     * @param desiredHeading the heading in radians
     */
    public void setDesiredHeading(double desiredHeading) {
        this.desiredHeading = desiredHeading;
    }

    /**
     * With the robot at (x, y), calculates the drive vector of the robot
     * in order to follow a parabola and arrive at the waypoint (wx, wy)
     * that is the parabola's vertex.
     * The parabola is defined to contain the robot's coordinates.
     *
     * @param wx the waypoint x coordinate
     * @param wy the waypoint y coordinate
     * @param toIntake whether the robot is going to the intake
     *
     * @return the drive vector in [x, y] notation
     */
    public double[] vectorToVertex(double wx, double wy, boolean toIntake) {
        if(x == wx)
            return toIntake ? new double[]{1.0, 0.0} : new double[]{-1.0, 0.0};
        return new double[]{wx - x, (wy - y) * 2.0};
    }

    /**
     * With the robot at (x, y), calculates the drive vector of the robot
     * in order to follow a parabola and arrive at the waypoint (wx, wy).
     * The parabola is defined with its vertex constrained to the x-value
     * of h (the previous waypoint), and the curve consists of both the
     * waypoint and robot coordinates.
     *
     * @param wx the waypoint x coordinate
     * @param wy the waypoint y coordinate
     * @param h  the x value of the previous waypoint
     * @param toIntake whether the robot is going to the intake
     * @return the drive vector in [x, y] notation
     */
    public double[] vectorFromVertex(double wx, double wy, double h, boolean toIntake) {
        if(x == wx)
            return toIntake ? new double[]{1.0, 0.0} : new double[]{-1.0, 0.0};

        double robotDistSquared = Math.pow(x - h, 2);
        double waypointDistSquared = Math.pow(wx - h, 2);

        if(robotDistSquared == waypointDistSquared)
            return y < wy ? new double[]{0.0, 1.0} : new double[]{0.0, -1.0};

        double k = (wy * robotDistSquared - y * waypointDistSquared) / (robotDistSquared - waypointDistSquared);
        return (x > wx) == toIntake ? new double[]{h - x, (k - y) * 2.0} : new double[]{x - h, (y - k) * 2.0};
    }

    /**
     * Automatically directs the robot to the Coordinates of the Correct Intake
     * area using parabolas in piecewise.
     *
     * @param turn the turn speed proportion
     * @param autoAlign whether to autoAlign
     * @param lowGear whether to drive in low gear
     */
    public void splineToIntake(double turn, boolean autoAlign, boolean lowGear) {
        double[] xy;
        if(x < LEFT_WAYPOINT_X)
            xy = vectorToVertex(LEFT_WAYPOINT_X, isBlueAlliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, true);
        else if(x < RIGHT_WAYPOINT_X)
            xy = vectorToVertex(RIGHT_WAYPOINT_X, isBlueAlliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, true);
        else
            xy = vectorFromVertex(INTAKE_X, isBlueAlliance ? BLUE_INTAKE_Y : RED_INTAKE_Y, RIGHT_WAYPOINT_X, true);

        double distance = Math.hypot(INTAKE_X - x, (isBlueAlliance ? BLUE_INTAKE_Y : RED_INTAKE_Y) - y);
        double powerRatio = (distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0) / Math.hypot(xy[0], xy[1]);
        drive(xy[0] * powerRatio, xy[1] * powerRatio, turn, autoAlign, lowGear);
    }

    /**
     * Automatically directs the robot to the Coordinates of the Correct Backstage
     * area using parabolas in piecewise.
     *
     * @param turn the turn speed proportion
     * @param autoAlign whether to autoAlign
     * @param scoringY the Y value to end at
     * @param lowGear whether to drive in low gear
     */
    public void splineToScoring(double turn, boolean autoAlign, double scoringY, boolean lowGear) {
        double[] xy;
        if(x > RIGHT_WAYPOINT_X)
            xy = vectorToVertex(RIGHT_WAYPOINT_X, WAYPOINT_Y_TO_SCORING, false);
        else if(x > LEFT_WAYPOINT_X)
            xy = vectorToVertex(LEFT_WAYPOINT_X, WAYPOINT_Y_TO_SCORING, false);
        else
            xy = vectorFromVertex(SCORING_X, scoringY, LEFT_WAYPOINT_X, false);

        double distance = Math.hypot(SCORING_X - x, scoringY - y);
        double powerRatio = (distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0) / Math.hypot(xy[0], xy[1]);
        drive(xy[0] * powerRatio, xy[1] * powerRatio, turn, autoAlign, lowGear);
    }

    /**
     * Updates Pose using Odometry Wheels
     */
    public void update() {
        int currentLeft = leftOdometry.getCurrentPosition();
        int currentRight = rightOdometry.getCurrentPosition();
        int currentFront = frontOdometry.getCurrentPosition();

        double deltaX = (currentLeft - previousLeftPosition + currentRight - previousRightPosition) *
                INCHES_PER_TICK * STRAFE_ODOMETRY_CORRECTION;
        double deltaY = (currentFront - previousFrontPosition) *
                INCHES_PER_TICK * FORWARD_ODOMETRY_CORRECTION;

        heading = imuOffset - getRawHeading();

        double headingRadians = Math.toRadians(heading);
        double prevHeadingRadians = Math.toRadians(previousHeading);

        double averagedHeadingRadians = Math.atan2(Math.sin(headingRadians) + Math.sin(prevHeadingRadians),
                Math.cos(headingRadians) + Math.cos(prevHeadingRadians));

        x += deltaX * Math.sin(averagedHeadingRadians) + deltaY * Math.cos(averagedHeadingRadians);
        y += -deltaX * Math.cos(averagedHeadingRadians) + deltaY * Math.sin(averagedHeadingRadians);

        previousLeftPosition = currentLeft;
        previousRightPosition = currentRight;
        previousFrontPosition = currentFront;
        previousHeading = heading;
    }

    /**
     * Sets the TeleopRobot Pose
     *
     * @param pose the x, y, theta of the robot
     */
    public void setPose(double[] pose) {
        x = pose[0];
        y = pose[1];
        imuOffset = pose[2] + getRawHeading();
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
     * Get Motor Velocities for telemetry
     *
     * @return left, right, back velocities
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
        return new double[]{previousLeftPosition, previousRightPosition, previousFrontPosition};
    }

    /**
     * Get the robot coordinates for telemetry
     *
     * @return the x and y value of the center of the bot
     */
    public double[] getXY() {
        return new double[]{x, y};
    }

    private double getRawHeading() {
        return Math.toRadians(navx.getYaw());
    }

    /**
     * Gets the heading of the robot on the field coordinate system
     *
     * @return the heading in degrees
     */
    public double getFieldHeading() {
        return AngleMath.addAnglesDegrees(Math.toDegrees(heading), 0.0);
    }
}