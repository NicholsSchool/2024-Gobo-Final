package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

//TODO: add all subsystems and controls
//TODO: edit all arm set positions

/**
 * Integrates Robot Subsystems and Controllers
 */
public class Robot {
    private final Controller driverOI;
    private final Controller operatorOI;
    private final Drivetrain drivetrain;
    private final Arm arm;
    private final Telemetry telemetry;
    private final ElapsedTime loopTimer;
    private final boolean isBlueAlliance;

    /**
     * Instantiates the Robot. Call during init()
     *
     * @param hardwareMap the hardware map
     * @param isBlueAlliance whether we are blue alliance
     * @param gamepad1  the driver gamepad
     * @param gamepad2  the operator gamepad
     * @param telemetry the telemetry
     * @param pose the robot's initial pose
     */
    public Robot(HardwareMap hardwareMap, boolean isBlueAlliance, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, double[] pose) {
        this.isBlueAlliance = isBlueAlliance;
        driverOI = new Controller(gamepad1);
        operatorOI = new Controller(gamepad2);
        drivetrain = new Drivetrain(hardwareMap, isBlueAlliance, pose[0], pose[1], pose[2]);
        arm = new Arm(hardwareMap);
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
        driverOI.update();
        operatorOI.update();
        drivetrain.update();
    }

    private void driverControls() {

    }

    private void operatorControls() {
        double armDesiredPosition;
        if(operatorOI.dpadUp.isPressed())
            armDesiredPosition = 2500;
        else if(operatorOI.dpadDown.isPressed())
            armDesiredPosition = 0.0;
        else if(operatorOI.dpadLeft.isPressed() && isBlueAlliance ||
                operatorOI.dpadRight.isPressed() && !isBlueAlliance)
            armDesiredPosition = 1250;
        else if(operatorOI.dpadLeft.isPressed() && !isBlueAlliance ||
                operatorOI.dpadRight.isPressed() && isBlueAlliance)
            armDesiredPosition = 1250;
        else
            armDesiredPosition = arm.getArmPosition();

        if(operatorOI.leftTrigger.getValue() > 0.0)
            arm.climb(-operatorOI.leftTrigger.getValue());
        else if(operatorOI.leftStickY.zeroLongEnough())
            arm.armGoToPosition(armDesiredPosition);
        else
            arm.shoulderManual(operatorOI.leftStickY.getValue());

        arm.wristManual(operatorOI.rightStickY.getValue());
        arm.launchPlane(operatorOI.back.isPressed());
    }

    private void outputTelemetry() {
        telemetry.addData("loop time", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
    }
}