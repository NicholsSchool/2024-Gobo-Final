package org.firstinspires.ftc.teamcode.subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.other.AngleMath;
import org.firstinspires.ftc.teamcode.other.CoordinateMotionProfile;
import org.firstinspires.ftc.teamcode.other.MotionProfile;
import org.firstinspires.ftc.teamcode.other.Constants.DriveConstants;

//TODO: test RUN_USING_ENCODER effect on loop time
//TODO: tune the drive motors ff OR remove drive encoders
//TODO: tune spline p
//TODO: tune spline error
//TODO: optimize spline waypoints
//TODO: tune the driving profile speed
//TODO: tune the odometry correction

/**
 * Robot Drivetrain Subsystem
 */
public class Drivetrain {
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

        double[] xy = driveProfile.update(xIn, yIn,
                (lowGear ? DriveConstants.LOW_GEAR : DriveConstants.HIGH_GEAR) - Math.abs(turn));
        double power = Math.hypot(xy[0], xy[1]);
        double angle = Math.toDegrees(Math.atan2(xy[1], xy[0]));

        leftDrive.setPower(turn +
                power * Math.cos(Math.toRadians(angle + DriveConstants.LEFT_DRIVE_OFFSET - heading)));
        rightDrive.setPower(turn +
                power * Math.cos(Math.toRadians(angle + DriveConstants.RIGHT_DRIVE_OFFSET - heading)));
        backDrive.setPower(turn +
                power * Math.cos(Math.toRadians(angle + DriveConstants.BACK_DRIVE_OFFSET - heading)));
    }

    private double turnToAngle() {
        double error = AngleMath.addAngles(heading, -desiredHeading);
        return Math.abs(error) < DriveConstants.AUTO_ALIGN_ERROR ? 0.0 : error * DriveConstants.AUTO_ALIGN_P;
    }

    /**
     * Sets the heading to auto-align to
     *
     * @param desiredHeading the heading in degrees
     */
    public void setDesiredHeading(double desiredHeading) {
        this.desiredHeading = desiredHeading;
    }

    private double[] vectorToVertex(double wx, double wy, boolean toIntake) {
        if(x == wx)
            return toIntake ? new double[]{1.0, 0.0} : new double[]{-1.0, 0.0};
        return new double[]{wx - x, 2.0 * (wy - y)};
    }

    private double[] vectorFromVertex(double wx, double wy, double h, boolean toIntake) {
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
        double[] xy;
        if(x < DriveConstants.LEFT_WAYPOINT_X)
            xy = vectorToVertex(DriveConstants.LEFT_WAYPOINT_X, isBlueAlliance ?
                    DriveConstants.BLUE_WAYPOINT_Y : DriveConstants.RED_WAYPOINT_Y, true);
        else if(x < DriveConstants.RIGHT_WAYPOINT_X)
            xy = vectorToVertex(DriveConstants.RIGHT_WAYPOINT_X, isBlueAlliance ?
                    DriveConstants.BLUE_WAYPOINT_Y : DriveConstants.RED_WAYPOINT_Y, true);
        else
            xy = vectorFromVertex(DriveConstants.INTAKE_X, isBlueAlliance ?
                    DriveConstants.BLUE_INTAKE_Y : DriveConstants.RED_INTAKE_Y,
                    DriveConstants.RIGHT_WAYPOINT_X, true);

        double distance = Math.hypot(DriveConstants.INTAKE_X - x,
                (isBlueAlliance ? DriveConstants.BLUE_INTAKE_Y : DriveConstants.RED_INTAKE_Y) - y);
        double power = distance >= DriveConstants.SPLINE_ERROR ? DriveConstants.SPLINE_P * distance : 0.0;
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
        double[] xy;
        if(x > DriveConstants.RIGHT_WAYPOINT_X)
            xy = vectorToVertex(DriveConstants.RIGHT_WAYPOINT_X, DriveConstants.WAYPOINT_Y_TO_SCORING, false);
        else if(x > DriveConstants.LEFT_WAYPOINT_X)
            xy = vectorToVertex(DriveConstants.LEFT_WAYPOINT_X, DriveConstants.WAYPOINT_Y_TO_SCORING, false);
        else
            xy = vectorFromVertex(DriveConstants.SCORING_X, scoringY, DriveConstants.LEFT_WAYPOINT_X, false);

        double distance = Math.hypot(DriveConstants.SCORING_X - x, scoringY - y);
        double power = distance >= DriveConstants.SPLINE_ERROR ? DriveConstants.SPLINE_P * distance : 0.0;
        double hypotenuse = Math.hypot(xy[0], xy[1]);
        drive(xy[0] * power / hypotenuse, xy[1] * power / hypotenuse, turn, autoAlign, lowGear);
    }

    /**
     * Updates Pose using Odometry Wheels
     */
    public void update() {
        int currentLeft = leftOdometry.getCurrentPosition();
        int currentRight = rightOdometry.getCurrentPosition();
        int currentFront = frontOdometry.getCurrentPosition();

        double deltaX = (currentLeft - previousLeftPosition + currentRight - previousRightPosition) *
                DriveConstants.INCHES_PER_TICK * DriveConstants.STRAFE_ODOMETRY_CORRECTION;
        double deltaY = currentFront - previousFrontPosition *
                DriveConstants.INCHES_PER_TICK * DriveConstants.FORWARD_ODOMETRY_CORRECTION;

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