package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hand;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

//TODO: edit all arm set positions
//TODO: remove telemetry at competition

/**
 * Integrates Robot Subsystems and Controllers
 */
public class Robot {
    private final Controller driverController;
    private final Controller operatorController;
    private final Drivetrain drivetrain;
    private final Arm arm;
    private Hand hand;
    private Lights lights;
    private Vision vision;
    private final Telemetry telemetry;
    private final ElapsedTime loopTimer;
    private final boolean isBlueAlliance;
    private boolean splineToIntake;
    private boolean splineToScoring;
    private double splineScoringY;

    /**
     * Instantiates the Robot. Call during init()
     *
     * @param hardwareMap the hardware map
     * @param isBlueAlliance whether we are blue alliance
     * @param gamepad1  the driver gamepad
     * @param gamepad2  the operator gamepad
     * @param telemetry the telemetry
     * @param pose the robot's initial pose [x, y, theta]
     */
    public Robot(HardwareMap hardwareMap, boolean isBlueAlliance, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, double[] pose) {
        this.isBlueAlliance = isBlueAlliance;

        driverController = new Controller(gamepad1);
        operatorController = new Controller(gamepad2);

        drivetrain = new Drivetrain(hardwareMap, isBlueAlliance, pose[0], pose[1], pose[2]);
        arm = new Arm(hardwareMap);
        hand = new Hand(hardwareMap);
        lights = new Lights(hardwareMap, isBlueAlliance);
        vision = new Vision(hardwareMap, pose[0], pose[1]);

        this.telemetry = telemetry;
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * Full Robot functionalities. Call in each loop()
     */
    public void update() {
        updateInstances();
        driverControls();
        operatorControls();
        outputTelemetry();
    }

    private void updateInstances() {
        driverController.update();
        operatorController.update();
        drivetrain.update();

        double[] visionPose = vision.update();
        if(visionPose != null)
            drivetrain.setPose(visionPose);
    }

    private void driverControls() {
        double x = driverController.leftStickX.getValue();
        double y = driverController.leftStickY.getValue();
        double turn = driverController.rightStickX.getValue();

        boolean lowGear = driverController.leftTrigger.getValue() > 0.0;

        if(lowGear){
            x *= Constants.DriveConstants.LOW_GEAR;
            y *= Constants.DriveConstants.LOW_GEAR;
        }
        else {
            x *= Constants.DriveConstants.HIGH_GEAR;
            y *= Constants.DriveConstants.HIGH_GEAR;
        }

        boolean autoAlign = driverController.rightStickX.zeroLongEnough();

        if(!autoAlign)
            drivetrain.setDesiredHeading(drivetrain.getFieldHeading());
        else if(driverController.y.wasJustPressed())
            drivetrain.setDesiredHeading(isBlueAlliance ? 90.0 : -90.0);
        else if(driverController.a.wasJustPressed())
            drivetrain.setDesiredHeading(isBlueAlliance ? -90.0 : 90.0);
        else if(driverController.b.wasJustPressed())
            drivetrain.setDesiredHeading(isBlueAlliance ? 0.0 : -180.0);
        else if(driverController.x.wasJustPressed())
            drivetrain.setDesiredHeading(isBlueAlliance ? -180.0 : 0.0);

        if(isBlueAlliance)
            blueSplineControls();
        else
            redSplineControls();

        if(x != 0.0 || y != 0.0 || driverController.leftStick.wasJustPressed()) {
            splineToScoring = false;
            splineToIntake = false;
        }

        if(splineToIntake) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            drivetrain.splineToIntake(turn, autoAlign, lowGear);
        }
        else if(splineToScoring) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            drivetrain.splineToScoring(turn, autoAlign, splineScoringY, lowGear);
        }
        else {
            lights.setDefaultPattern();
            drivetrain.drive(x, y, turn, autoAlign, lowGear);
        }
    }

    private void blueSplineControls() {
        if(driverController.dpadLeft.wasJustPressed()) {
            splineScoringY = Constants.DriveConstants.BLUE_SCORING_Y_MID;
            splineToScoring = true;
        }
        else if(driverController.dpadRight.wasJustPressed())
            splineToIntake = true;

        else if(driverController.dpadUp.wasJustPressed()) {
            splineScoringY = Constants.DriveConstants.BLUE_SCORING_Y_FAR;
            splineToScoring = true;
        }
        else if(driverController.dpadDown.wasJustPressed()) {
            splineScoringY = Constants.DriveConstants.BLUE_SCORING_Y_CLOSE;
            splineToScoring = true;
        }
    }

    private void redSplineControls() {
        if(driverController.dpadLeft.wasJustPressed())
            splineToIntake = true;

        else if(driverController.dpadRight.wasJustPressed()) {
            splineScoringY = Constants.DriveConstants.RED_SCORING_Y_MID;
            splineToScoring = true;
        }
        else if(driverController.dpadUp.wasJustPressed()) {
            splineScoringY = Constants.DriveConstants.RED_SCORING_Y_FAR;
            splineToScoring = true;
        }
        else if(driverController.dpadDown.wasJustPressed()) {
            splineScoringY = Constants.DriveConstants.RED_SCORING_Y_CLOSE;
            splineToScoring = true;
        }
    }

    private void operatorControls() {
        double armDesiredPosition;
        if(operatorController.dpadUp.isPressed())
            armDesiredPosition = 2500;
        else if(operatorController.dpadDown.isPressed())
            armDesiredPosition = 0.0;
        else if(operatorController.dpadLeft.isPressed() && isBlueAlliance ||
                operatorController.dpadRight.isPressed() && !isBlueAlliance)
            armDesiredPosition = 1250;
        else if(operatorController.dpadLeft.isPressed() && !isBlueAlliance ||
                operatorController.dpadRight.isPressed() && isBlueAlliance)
            armDesiredPosition = 1250;
        else
            armDesiredPosition = arm.getArmPosition();

        if(operatorController.leftTrigger.getValue() > 0.0)
            arm.climb(-operatorController.leftTrigger.getValue());
        else if(operatorController.leftStickY.zeroLongEnough())
            arm.armGoToPosition(armDesiredPosition);
        else
            arm.shoulderManual(Constants.ArmConstants.SHOULDER_MAX *
                    operatorController.leftStickY.getValue());

        arm.wristManual(Constants.ArmConstants.WRIST_MAX *
                operatorController.rightStickY.getValue());

        arm.launchPlane(operatorController.back.isPressed());


        hand.leftGrabber(operatorController.leftTrigger.getValue() < 0.5);
        hand.rightGrabber(operatorController.rightTrigger.getValue() < 0.5);
    }

    private void outputTelemetry() {
        telemetry.addData("loop time", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
    }
}