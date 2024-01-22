package org.firstinspires.ftc.teamcode.teleops.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.TeleopConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

/**
 * A teleop for testing Drivetrain functionalities
 */
@Config
@TeleOp(name="[DASHBOARD] Drivetrain Testing")
public class DrivetrainTeleop extends OpMode implements TeleopConstants, DriveConstants {
    private ElapsedTime loopTimer;
    private Controller driverController;
    private Drivetrain drivetrain;
    private Lights lights;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private double[] alignAngles;
    private boolean splineToScoring;
    private boolean splineToIntake;
    private double splineScoringY;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        driverController = new Controller(gamepad1);
        drivetrain = new Drivetrain(hardwareMap, IS_BLUE_ALLIANCE, -48.0, -48.0, 90.0);
        lights = new Lights(hardwareMap, IS_BLUE_ALLIANCE);

        alignAngles = new double[]{90.0, -90.0, 0.0, -180.0};


        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drivetrain.update();
        driverController.update();

        double x = driverController.leftStickX.getValue();
        double y = driverController.leftStickY.getValue();
        double turn = driverController.rightStickX.getValue();

        boolean lowGear = driverController.leftTrigger.getValue() <= 0.5;

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

        blueSplineControls();

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
            lights.setAllianceColor();
            drivetrain.drive(x, y, turn, autoAlign, lowGear);
        }

        double[] xy = drivetrain.getXY();

        TelemetryPacket packet = new TelemetryPacket(true);
        packet.fieldOverlay()
                .setFill("green")
                .fillCircle(-xy[0], -xy[1], 18.0);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("robot x", xy[0]);
        telemetry.addData("robot y", xy[1]);
        telemetry.addData("theta", drivetrain.getFieldHeading());

        telemetry.addData("loop time millis", loopTimer.time());
        loopTimer.reset();
        telemetry.update();
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
}
