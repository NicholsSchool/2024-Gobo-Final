package org.firstinspires.ftc.teamcode.subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.other.AngleMath;
import org.firstinspires.ftc.teamcode.other.CoordinateMotionProfile;
import org.firstinspires.ftc.teamcode.other.MotionProfile;

//TODO: test RUN_USING_ENCODER effect on loop time
//TODO: tune the drive motors ff OR remove drive encoders
//TODO: debug the driving profile change
//TODO: debug the spline change
//TODO: tune spline p
//TODO: tune spline error
//TODO: optimize spline waypoints
//TODO: tune the driving profile speed
//TODO: tune the odometry correction

/**
 * Robot Drivetrain Subsystem
 */
public class Drivetrain {
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

    private final int MAX_MOTOR_VEL = 2800;
    private final double SPLINE_P = 0.025;
    private final double SPLINE_ERROR = 2.0;
    private final double LEFT_WAYPOINT_X = -14.0;
    private final double RIGHT_WAYPOINT_X = 38.0;

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
     * @param initialHeading the initial value for the robot heading
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
     * @param xIn the x input value
     * @param yIn the y input value
     * @param turn the turning power proportion
     * @param autoAlign whether to autoAlign
     * @param lowGear whether to put the robot to virtual low gear
     */
    public void drive(double xIn, double yIn, double turn, boolean autoAlign, boolean lowGear) {
        turn = turnProfile.update(autoAlign ? turnToAngle() : turn);

        double[] xy = driveProfile.update(xIn, yIn, (lowGear ? 0.45 : 0.9) - turn);
        double power = Math.hypot(xy[0], xy[1]);
        double angle = Math.toDegrees(Math.atan2(xy[1], xy[0]));

        leftDrive.setPower(turn + power * Math.cos(Math.toRadians(angle + 30.0 - heading)));
        rightDrive.setPower(turn + power * Math.cos(Math.toRadians(angle + 150.0 - heading)));
        backDrive.setPower(turn + power * Math.cos(Math.toRadians(angle + 270.0 - heading)));
    }

    /**
     * Aligns the robot to the desired heading by spinning anchor-less
     *
     * @return the turning speed as a proportion
     */
    public double turnToAngle() {
        double error = AngleMath.addAngles(heading, -desiredHeading);
        return Math.abs(error) < 0.5 ? 0.0 : error * 0.005;
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
     * @return the x and y components of the drive vector
     */
    public double[] vectorToVertex(double wx, double wy, boolean toIntake) {
        if(x == wx)
            return toIntake ? new double[]{1.0, 0.0} : new double[]{-1.0, 0.0};
        return new double[]{wx - x, 2.0 * (wy - y)};
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
     *
     * @return the x and y components of the drive vector
     */
    public double[] vectorFromVertex(double wx, double wy, double h, boolean toIntake) {
        if(x == wx)
            return toIntake ? new double[]{1.0, 0.0} : new double[]{-1.0, 0.0};

        double robotDistSquared = Math.pow(x - h, 2);
        double waypointDistSquared = Math.pow(wx - h, 2);

        if(robotDistSquared == waypointDistSquared)
            return y < wy ? new double[]{0.0, 1.0} : new double[]{0.0, -1.0};

        double k = (wy * robotDistSquared - y * waypointDistSquared) / (robotDistSquared - waypointDistSquared);
        return new double[]{h - x, 2.0 * (k - y)};
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
        double INTAKE_X = 56.0;
        double BLUE_INTAKE_Y = 56.0;
        double RED_INTAKE_Y = -56.0;
        double BLUE_WAYPOINT_Y = 36.0;
        double RED_WAYPOINT_Y = -36.0;

        double[] xy;
        if(x < LEFT_WAYPOINT_X)
            xy = vectorToVertex(LEFT_WAYPOINT_X, isBlueAlliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, true);
        else if(x < RIGHT_WAYPOINT_X)
            xy = vectorToVertex(RIGHT_WAYPOINT_X, isBlueAlliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y, true);
        else
            xy = vectorFromVertex(INTAKE_X, isBlueAlliance ? BLUE_INTAKE_Y : RED_INTAKE_Y, RIGHT_WAYPOINT_X, true);

        double distance = Math.hypot(INTAKE_X - x, (isBlueAlliance ? BLUE_INTAKE_Y : RED_INTAKE_Y) - y);
        double power = distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0;
        double hypotenuse = Math.hypot(xy[0], xy[1]);
        drive(xy[0] * power / hypotenuse, xy[1] * power / hypotenuse, turn, autoAlign, lowGear);
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
        final double SCORING_X = -42.0;
        final double WAYPOINT_Y = 0.0;

        double[] xy;
        if(x > RIGHT_WAYPOINT_X)
            xy = vectorToVertex(RIGHT_WAYPOINT_X, WAYPOINT_Y, false);
        else if(x > LEFT_WAYPOINT_X)
            xy = vectorToVertex(LEFT_WAYPOINT_X, WAYPOINT_Y, false);
        else
            xy = vectorFromVertex(SCORING_X, scoringY, LEFT_WAYPOINT_X, false);

        double distance = Math.hypot(SCORING_X - x, scoringY - y);
        double power = distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0;
        double hypotenuse = Math.hypot(xy[0], xy[1]);
        drive(xy[0] * power / hypotenuse, xy[1] * power / hypotenuse, turn, autoAlign, lowGear);
    }

    /**
     * Updates Pose using Odometry Wheels
     */
    public void update() {
        final int TICKS_PER_REV = 8192;
        final double DEAD_WHEEL_DIAMETER = 2.5;
        final double INCHES_PER_TICK = DEAD_WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;
        final double STRAFE_ODOMETRY_CORRECTION = 1.0;
        final double FORWARD_ODOMETRY_CORRECTION = 1.0;

        int currentLeft = leftOdometry.getCurrentPosition();
        int currentRight = rightOdometry.getCurrentPosition();
        int currentFront = frontOdometry.getCurrentPosition();

        double deltaX = (currentLeft - previousLeftPosition + currentRight - previousRightPosition) *
                INCHES_PER_TICK * STRAFE_ODOMETRY_CORRECTION;
        double deltaY = currentFront - previousFrontPosition *
                INCHES_PER_TICK * FORWARD_ODOMETRY_CORRECTION;

        heading = imuOffset - navx.getYaw();

        double adjustedHeadingRadians = Math.toRadians((heading + previousHeading) * 0.5);

        x += deltaX * Math.sin(adjustedHeadingRadians) + deltaY * Math.cos(adjustedHeadingRadians);
        y += -deltaX * Math.cos(adjustedHeadingRadians) + deltaY * Math.sin(adjustedHeadingRadians);

        previousLeftPosition = currentLeft;
        previousRightPosition = currentRight;
        previousFrontPosition = currentFront;
        previousHeading = heading;
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
     * Sets the Drive Wheels to the RUN-WITHOUT_ENCODER run mode
     */
    public void disableDriveEncoders() {
        leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the Robot Pose
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

    /**
     * Gets the heading of the robot on the field coordinate system
     *
     * @return the heading in degrees
     */
    public double getFieldHeading() {
        return AngleMath.addAngles(heading, 0.0);
    }
}