package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.Controller;

//TODO: add subsystems and controls

/**
 * Integrates Robot Subsystems and Controllers
 */
public class Robot {
    private final Controller driverOI;
    private final Controller operatorOI;
    private final Telemetry telemetry;
    private final ElapsedTime loopTimer;

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
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * Full Robot functionalities. Call in each loop()
     */
    public void update() {
        updatePrior();

        teleopLogic();

        outputTelemetry();

        updateAfter();
    }

    private void updatePrior() {
        driverOI.update();
        operatorOI.update();
    }

    private void teleopLogic() {

    }

    private void outputTelemetry() {
        telemetry.addData("loop time", loopTimer.time());
        telemetry.update();
    }

    private void updateAfter() {
        loopTimer.reset();
    }
}