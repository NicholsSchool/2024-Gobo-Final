package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.other.CoordinateTrapezoidProfile;
import org.firstinspires.ftc.teamcode.other.TrapezoidProfile;

/**
 * The Controller containing Button and Axis objects
 */
public class Controller {
    private final Gamepad gamepad;

    private final CoordinateTrapezoidProfile leftJoystickProfile;
    private final TrapezoidProfile rightXProfile;

    public Button leftBumper;
    public Button rightBumper;
    public Button dpadUp;
    public Button dpadDown;
    public Button dpadLeft;
    public Button dpadRight;
    public Button a;
    public Button b;
    public Button x;
    public Button y;
    public Button back;
    public Button start;
    public Button leftStick;
    public Button rightStick;
    
    public Axis leftTrigger;
    public Axis rightTrigger;
    public Axis leftStickX;
    public Axis leftStickY;
    public Axis rightStickX;
    public Axis rightStickY;

    /**
     * Instantiates the Controller
     * 
     * @param gamepad the gamepad to poll
     */
    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;

        leftJoystickProfile = new CoordinateTrapezoidProfile();
        rightXProfile =  new TrapezoidProfile();
        
        leftBumper = new Button();
        rightBumper = new Button();
        dpadUp = new Button();
        dpadDown = new Button();
        dpadLeft = new Button();
        dpadRight = new Button();
        a = new Button();
        b = new Button();
        x = new Button();
        y = new Button();
        back = new Button();
        start = new Button();
        leftStick = new Button();
        rightStick = new Button();

        leftTrigger = new Axis();
        rightTrigger = new Axis();
        leftStickX = new Axis();
        leftStickY = new Axis();
        rightStickX = new Axis();
        rightStickY = new Axis();
    }

    /**
     * Updates all instances.
     */
    public void update() {
        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);
        dpadUp.update(gamepad.dpad_up);
        dpadDown.update(gamepad.dpad_down);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        back.update(gamepad.back);
        start.update(gamepad.start);
        leftStick.update(gamepad.left_stick_button);
        rightStick.update(gamepad.right_stick_button);

        double[] leftStick = leftJoystickProfile.update(gamepad.left_stick_x, -gamepad.left_stick_y);
        leftStickX.update(leftStick[0]);
        leftStickY.update(leftStick[1]);
        rightStickX.update(rightXProfile.update(gamepad.right_stick_x));
        rightStickY.update(-gamepad.right_stick_y);
        leftTrigger.update(gamepad.left_trigger);
        rightTrigger.update(gamepad.right_trigger);
    }

    /**
     * The Radius Value of the Left Joystick
     *
     * @return the hypotenuse length of x and y
     */
    public double leftStickRadius() {
        return Range.clip(Math.hypot(leftStickX.getValue(), leftStickY.getValue()), 0.0, 1.0);
    }

    /**
     * The Theta Value of the Left Joystick
     *
     * @return the angle in degrees
     */
    public double leftStickTheta() {
        return Math.toDegrees(Math.atan2(leftStickY.getValue(), leftStickX.getValue()));
    }
}