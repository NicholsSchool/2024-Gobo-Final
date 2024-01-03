package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//TODO: Fully Test IMU

//TODO: Test out the NavX

/**
 * A teleop for testing the internal BHI260IMU
 */
@Config
@TeleOp(name="Internal IMU Testing")
public class ImuTeleop extends OpMode {
    private ElapsedTime loopTimer;
    private BHI260IMU imu;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("loop time millis", loopTimer.time());
        telemetry.addData("imu", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        loopTimer.reset();
    }
}
