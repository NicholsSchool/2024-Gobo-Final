package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.other.AngleMath;
import org.firstinspires.ftc.teamcode.other.CoordinateTrapezoidProfile;
import org.firstinspires.ftc.teamcode.other.TrapezoidProfile;

//TODO: edit scoring and intaking waypoints
//TODO: odometry tuning
//TODO: tune both profiles
//TODO: drive motor tuning
//TODO: why is the loop time so high?

/**
 * Robot Drivetrain Subsystem
 */
public class Drivetrain {
    public final double BLUE_SCORING_Y_CLOSE = -42.0;
    public final double BLUE_SCORING_Y_MED = -36.0;
    public final double BLUE_SCORING_Y_FAR = -30.0;
    public final double RED_SCORING_Y_CLOSE = 42.0;
    public final double RED_SCORING_Y_MED = 36.0;
    public final double RED_SCORING_Y_FAR = 30.0;

    private final int MAX_MOTOR_VEL = 2800;
    private final double SPLINE_P = 0.05;
    private final double SPLINE_ERROR = 2.0;
    private final double SPLINE_GOVERNOR = 1.0;
    private final double LEFT_WAYPOINT_X = -14.0;
    private final double RIGHT_WAYPOINT_X = 38.0;
    private final double BLUE_WAYPOINT_Y = 36.0;
    private final double RED_WAYPOINT_Y = -36.0;

    private final boolean isBlueAlliance;
    private final DcMotorEx leftDrive, rightDrive, backDrive, frontOdometry, leftOdometry, rightOdometry;
    private final BHI260IMU imu;
    private int previousLeftPosition, previousRightPosition, previousFrontPosition;
    private double x, y, heading, imuOffset, desiredHeading;
    private final CoordinateTrapezoidProfile driveProfile;
    private final CoordinateTrapezoidProfile positionProfile;
    private final TrapezoidProfile turnProfile;
    private final TrapezoidProfile headingProfile;

    /**
     * Initializes the Drivetrain subsystem
     *
     * @param hwMap the hardwareMap
     * @param isBlueAlliance true for blue, false for red
     * @param x the initial x coordinate
     * @param y the initial y coordinate
     * @param imuOffset the initial value for the heading
     */
    public Drivetrain(HardwareMap hwMap, boolean isBlueAlliance, double x, double y, double imuOffset) {
        this.isBlueAlliance = isBlueAlliance;
        this.x = x;
        this.y = y;
        this.imuOffset = imuOffset;
        this.heading = imuOffset;
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
//        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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

        imu = hwMap.get(BHI260IMU.class, "imu");
        imu.initialize(new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();

        driveProfile = new CoordinateTrapezoidProfile();
        turnProfile = new TrapezoidProfile();
        positionProfile = new CoordinateTrapezoidProfile(x, y, -72.0, 72.0, 60.0);
        headingProfile = new TrapezoidProfile(imuOffset, -1000.0, 1000.0, 720.0);
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
     * Drives the robot with full functionality
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

        power = Range.clip(power, turn - 1.0, 1.0 - turn);
        double angleRadians = Math.toRadians(angle);
        double[] xy = driveProfile.update(power * Math.cos(angleRadians), power * Math.sin(angleRadians));
        power = Range.clip(Math.hypot(xy[0], xy[1]), 0.0, 1.0);
        angle = Math.atan2(xy[1], xy[0]);

        leftDrive.setVelocity((turn + power * Math.cos(Math.toRadians(angle - 60.0 + 90.0 - heading))) * MAX_MOTOR_VEL);
        rightDrive.setVelocity((turn + power * Math.cos(Math.toRadians(angle + 60.0 + 90.0 - heading))) * MAX_MOTOR_VEL);
        backDrive.setVelocity((turn + power * Math.cos((Math.toRadians(angle + 180.0 + 90.0 - heading)))) * MAX_MOTOR_VEL);
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
        return Range.clip(error * 0.025, -0.25, 0.25);
    }

    /**
     * Sets the heading to auto-align to
     *
     * @param desiredHeading the heading in degrees [-180, 180)
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
        double INTAKE_X = 60.0;
        double BLUE_INTAKE_Y = 60.0;
        double RED_INTAKE_Y = -60.0;

        double distance = Math.sqrt(Math.pow(INTAKE_X - x, 2) +
                Math.pow(isBlueAlliance ? BLUE_INTAKE_Y - y : RED_INTAKE_Y - y, 2) );
        double power = distance >= SPLINE_ERROR ?
                Range.clip(SPLINE_P * distance, -SPLINE_GOVERNOR, SPLINE_GOVERNOR) : 0.0;

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
        double SCORING_X = -44.0;

        double distance = Math.sqrt(Math.pow(SCORING_X - x, 2) +
                Math.pow(scoringY - y, 2) );
        double power = distance >= SPLINE_ERROR ?
                Range.clip(SPLINE_P * distance, -SPLINE_GOVERNOR, SPLINE_GOVERNOR) : 0.0;

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

        heading = AngleMath.addAngles(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), imuOffset);

        int currentLeft = leftOdometry.getCurrentPosition();
        int currentRight = rightOdometry.getCurrentPosition();
        int currentFront = frontOdometry.getCurrentPosition();

        int deltaLeft = currentLeft - previousLeftPosition;
        int deltaRight = currentRight - previousRightPosition;
        int deltaFront = currentFront - previousFrontPosition;

        double deltaX = (deltaLeft + deltaRight) * INCHES_PER_TICK * STRAFE_ODOMETRY_CORRECTION;
        double deltaY = deltaFront * INCHES_PER_TICK * FORWARD_ODOMETRY_CORRECTION;

        double inRadians = Math.toRadians(heading);
        y += -deltaX * Math.cos(inRadians) + deltaY * Math.sin(inRadians);
        x += deltaX * Math.sin(inRadians) + deltaY * Math.cos(inRadians);

        previousLeftPosition = currentLeft;
        previousRightPosition = currentRight;
        previousFrontPosition = currentFront;
    }

    /**
     * Sets the Robot Pose using the IMU offset
     *
     * @param pose the x, y, theta of the robot
     */
    public void setPose(double[] pose) {
        double[] xy = positionProfile.update(pose[0], pose[1]);
        x = xy[0];
        y = xy[1];

        imuOffset = AngleMath.addAngles(headingProfile.update(
                pose[2] - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), 0.0);
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
        return heading;
    }
}