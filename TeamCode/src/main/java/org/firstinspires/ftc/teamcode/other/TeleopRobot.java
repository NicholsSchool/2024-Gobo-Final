package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hand;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

//TODO: remove telemetry method after checking final loop time

/**
 * Integrates TeleopRobot Subsystems and Controllers
 */
public class TeleopRobot implements DriveConstants, ArmConstants {
    private final Controller driverController;
    private final Controller operatorController;
    private final Drivetrain drivetrain;
    private final Arm arm;
    private final Hand hand;
    private final Lights lights;
    private final Vision vision;
    private final Telemetry telemetry;
    private final ElapsedTime loopTimer;
    private final ElapsedTime planeTimer;
    private final boolean isBlueAlliance;
    private final double[] alignAngles;
    private boolean splineToIntake;
    private boolean splineToScoring;
    private double splineScoringY;
    private double armDesiredPosition;
    private boolean leftClamp;
    private boolean rightClamp;

    /**
     * Instantiates the TeleopRobot. Call during init()
     *
     * @param hwMap the hardware map
     * @param isBlue whether we are blue alliance
     * @param g1  the driver gamepad
     * @param g2  the operator gamepad
     * @param telemetry the telemetry
     * @param pose the robot's initial pose [x, y, theta]
     */
    public TeleopRobot(HardwareMap hwMap, boolean isBlue, Gamepad g1, Gamepad g2, Telemetry telemetry, double[] pose) {
        this.isBlueAlliance = isBlue;

        driverController = new Controller(g1);
        operatorController = new Controller(g2);

        drivetrain = new Drivetrain(hwMap, isBlue, pose[0], pose[1], pose[2]);
        arm = new Arm(hwMap);
        hand = new Hand(hwMap);
        lights = new Lights(hwMap, isBlue);
        vision = new Vision(hwMap, pose[0], pose[1]);

        leftClamp = true;
        rightClamp = true;

        alignAngles = isBlue ? new double[]{90.0, -90.0, 0.0, -180.0} : new double[]{-90.0, 90.0, -180.0, 0.0};

        this.telemetry = telemetry;
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        planeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    /**
     * Full TeleopRobot functionalities. Call in each loop()
     */
    public void update() {
        updateInstances();
        driverControls();
        operatorControls();
        signalLights();
        outputTelemetry();
    }

    private void updateInstances() {
        driverController.update();
        operatorController.update();

        double[] visionPose;
        if(driverController.back.isPressed()) {
            visionPose = vision.update();
            if(visionPose != null){
                drivetrain.setPose(visionPose);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        }

        drivetrain.update();
    }

    private void driverControls() {
        double x = driverController.leftStickX.getValue();
        double y = driverController.leftStickY.getValue();
        double turn = driverController.rightStickX.getValue();

        boolean lowGear = driverController.leftTrigger.getValue() <= 0.5;

        if(!isBlueAlliance) {
            x *= -1;
            y *= -1;
        }

        if(lowGear) {
            x *= LOW_GEAR;
            y *= LOW_GEAR;
        }
        else {
            x *= HIGH_GEAR;
            y *= HIGH_GEAR;
        }

        boolean autoAlign = driverController.rightStickX.zeroLongEnough();

        if(!autoAlign)
            drivetrain.setDesiredHeading(drivetrain.getFieldHeading());
        else if(driverController.y.wasJustPressed())
            drivetrain.setDesiredHeading(alignAngles[0]);
        else if(driverController.a.wasJustPressed())
            drivetrain.setDesiredHeading(alignAngles[1]);
        else if(driverController.b.wasJustPressed())
            drivetrain.setDesiredHeading(alignAngles[2]);
        else if(driverController.x.wasJustPressed())
            drivetrain.setDesiredHeading(alignAngles[3]);

        if(isBlueAlliance)
            blueSplineControls();
        else
            redSplineControls();

        if(x != 0.0 || y != 0.0 || driverController.leftStick.wasJustPressed()) {
            splineToScoring = false;
            splineToIntake = false;
        }

        if(splineToIntake)
            drivetrain.splineToIntake(turn, autoAlign, lowGear);
        else if(splineToScoring)
            drivetrain.splineToScoring(turn, autoAlign, splineScoringY, lowGear);
        else
            drivetrain.drive(x, y, turn, autoAlign, lowGear);
    }

    private void blueSplineControls() {
        if(driverController.dpadLeft.wasJustPressed()) {
            splineScoringY = BLUE_SCORING_Y_MID;
            splineToScoring = true;
        }
        else if(driverController.dpadRight.wasJustPressed())
            splineToIntake = true;

        else if(driverController.dpadUp.wasJustPressed()) {
            splineScoringY = BLUE_SCORING_Y_FAR;
            splineToScoring = true;
        }
        else if(driverController.dpadDown.wasJustPressed()) {
            splineScoringY = BLUE_SCORING_Y_CLOSE;
            splineToScoring = true;
        }
    }

    private void redSplineControls() {
        if(driverController.dpadLeft.wasJustPressed())
            splineToIntake = true;

        else if(driverController.dpadRight.wasJustPressed()) {
            splineScoringY = RED_SCORING_Y_MID;
            splineToScoring = true;
        }
        else if(driverController.dpadUp.wasJustPressed()) {
            splineScoringY = RED_SCORING_Y_FAR;
            splineToScoring = true;
        }
        else if(driverController.dpadDown.wasJustPressed()) {
            splineScoringY = RED_SCORING_Y_CLOSE;
            splineToScoring = true;
        }
    }

    private void operatorControls() {
        if(operatorController.start.wasJustPressed())
            arm.resetEncoder();

        if(operatorController.dpadUp.isPressed())
            armDesiredPosition = 2420.0;
        else if(operatorController.dpadDown.isPressed())
            armDesiredPosition = 220.0;
        else if(operatorController.dpadLeft.isPressed() && isBlueAlliance ||
                operatorController.dpadRight.isPressed() && !isBlueAlliance)
            armDesiredPosition = 1220.0;
        else if(operatorController.dpadLeft.isPressed() && !isBlueAlliance ||
                operatorController.dpadRight.isPressed() && isBlueAlliance)
            armDesiredPosition = 1520;

        if(operatorController.leftTrigger.getValue() > 0.0) {
            arm.climb(-operatorController.leftTrigger.getValue());
            armDesiredPosition = arm.getArmPosition();
        }
        else if(operatorController.leftStickY.zeroLongEnough())
            arm.armGoToPosition(armDesiredPosition);
        else {
            arm.armManual(SHOULDER_MAX * operatorController.leftStickY.getValue());
            armDesiredPosition = arm.getArmPosition();
        }

        arm.wristManual(WRIST_MAX * operatorController.rightStickY.getValue());


        if(operatorController.back.isPressed()) {
            double alignAngle;
            if(isBlueAlliance)
                alignAngle = alignAngles[2];
            else
                alignAngle = alignAngles[3];

            armDesiredPosition = 1520;
            drivetrain.setDesiredHeading(alignAngle);
            if(Math.abs(drivetrain.getFieldHeading()) <= 4.0 * AUTO_ALIGN_ERROR) {
                arm.launchPlane(true);
                if(planeTimer.time() > 1.0)
                    arm.launchPlane(false);
            }
            else
                planeTimer.reset();
        }

        boolean aIsPressed = operatorController.a.wasJustPressed();
        if(operatorController.x.wasJustPressed() || aIsPressed){
            leftClamp = !leftClamp;
        }
        if(operatorController.b.wasJustPressed() || aIsPressed){
            rightClamp = !rightClamp;
        }

        hand.leftGrabber(leftClamp);
        hand.rightGrabber(rightClamp);
    }

    private void signalLights() {
        if(splineToIntake)
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        else if(splineToScoring)
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        else
            lights.setAllianceColor();
    }

    private void outputTelemetry() {
        telemetry.addData("imu", drivetrain.getFieldHeading());
        telemetry.addData("x", drivetrain.getXY()[0]);
        telemetry.addData("y",drivetrain.getXY()[1]);
        telemetry.addData("arm", arm.getArmPosition());
        telemetry.addData("loop time", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
    }
}