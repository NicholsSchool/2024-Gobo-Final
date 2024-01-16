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

//TODO: edit all arm set positions: floor, scoring, plane launching, climbing
//TODO: remove telemetry method after checking final loop time

/**
 * Integrates Robot Subsystems and Controllers
 */
public class Robot implements DriveConstants, ArmConstants {
    private final Controller driverController;
    private final Controller operatorController;
    private final Drivetrain drivetrain;
    private final Arm arm;
    private final Hand hand;
    private final Lights lights;
    private final Vision vision;
    private final Telemetry telemetry;
    private final ElapsedTime loopTimer;
    private final boolean isBlueAlliance;
    private final double[] alignAngles;
    private boolean splineToIntake;
    private boolean splineToScoring;
    private double splineScoringY;

    /**
     * Instantiates the Robot. Call during init()
     *
     * @param hwMap the hardware map
     * @param isBlue whether we are blue alliance
     * @param g1  the driver gamepad
     * @param g2  the operator gamepad
     * @param telemetry the telemetry
     * @param pose the robot's initial pose [x, y, theta]
     */
    public Robot(HardwareMap hwMap, boolean isBlue, Gamepad g1, Gamepad g2, Telemetry telemetry, double[] pose) {
        this.isBlueAlliance = isBlue;

        driverController = new Controller(g1);
        operatorController = new Controller(g2);

        drivetrain = new Drivetrain(hwMap, isBlue, pose[0], pose[1], pose[2]);
        arm = new Arm(hwMap);
        hand = new Hand(hwMap);
        lights = new Lights(hwMap, isBlue);
        vision = new Vision(hwMap, pose[0], pose[1]);

        alignAngles = isBlue ? new double[]{90.0, -90.0, 0.0, -180.0} : new double[]{-90.0, 90.0, -180.0, 0.0};

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
        drivetrain.setPose(vision.update());
        drivetrain.update();
    }

    private void driverControls() {
        double x = driverController.leftStickX.getValue();
        double y = driverController.leftStickY.getValue();
        double turn = driverController.rightStickX.getValue();

        boolean lowGear = driverController.leftTrigger.getValue() > 0.5;

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

        if(splineToIntake) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            drivetrain.splineToIntake(turn, autoAlign, lowGear);
        }
        else if(splineToScoring) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            drivetrain.splineToScoring(turn, autoAlign, splineScoringY, lowGear);
        }
        else {
            lights.setDefaultPattern();
            drivetrain.drive(x, y, turn, autoAlign, lowGear);
        }
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
            arm.shoulderManual(SHOULDER_MAX * operatorController.leftStickY.getValue());

        arm.wristManual(WRIST_MAX * operatorController.rightStickY.getValue());

        arm.launchPlane(operatorController.back.isPressed());

        hand.leftGrabber(operatorController.leftBumper.isPressed());
        hand.rightGrabber(operatorController.rightBumper.isPressed());
    }

    private void outputTelemetry() {
        telemetry.addData("loop time", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
    }
}