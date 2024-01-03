package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.Controller;

/**
 * Integrates Robot Subsystems and Controllers
 */
public class Robot {
    private final Controller driverOI;
    private final Controller operatorOI;
    private final Telemetry telemetry;

    /**
     * Instantiates the Robot. Call during init()
     *
     * @param gamepad1  the driver gamepad
     * @param gamepad2  the operator gamepad
     * @param telemetry the telemetry
     */
    public Robot(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        driverOI = new Controller(gamepad1);
        operatorOI = new Controller(gamepad2);
        this.telemetry = telemetry;
    }

    /**
     * Full Robot functionalities. Call in each loop()
     */
    public void update() {
        updatePrior();
        teleopLogic();
        updateAfter();
        outputTelemetry();
    }

    private void updatePrior() {
        driverOI.update();
        operatorOI.update();
    }

    private void teleopLogic() {

    }

    private void updateAfter() {
        //TODO: arm update here
    }

    private void outputTelemetry() {
        telemetry.update();
    }
}