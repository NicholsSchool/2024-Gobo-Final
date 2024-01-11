package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * The Controller containing Button and Axis objects
 */
public class Controller {
    private final Gamepad gamepad;

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

        leftStickX.update(gamepad.left_stick_x);
        leftStickY.update(-gamepad.left_stick_y);
        rightStickX.update(gamepad.right_stick_x);
        rightStickY.update(-gamepad.right_stick_y);
        leftTrigger.update(gamepad.left_trigger);
        rightTrigger.update(gamepad.right_trigger);
    }
}