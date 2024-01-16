package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;

/**
 * A Button on a Controller
 */
public class Button {
    private boolean isPressed;
    private boolean wasPressed;
    private boolean toggleState;

    /**
     * Instantiates the Button
     */
    public Button() {}

    /**
     * Updates the Button with the new state
     *
     * @param isNowPressed whether the button is pressed
     */
    public void update(boolean isNowPressed) {
        wasPressed = isPressed;
        isPressed = isNowPressed;
        toggleState = wasJustPressed() != toggleState;
    }

    /**
     * Whether the button is pressed
     *
     * @return the button's current state
     */
    public boolean isPressed() {
        return isPressed;
    }

    /**
     * Whether the button was just pressed
     *
     * @return true iff the button's state changed to true
     */
    public boolean wasJustPressed() {
        return isPressed && !wasPressed;
    }

    /**
     * The Button's toggle state switches when the Button was just pressed
     *
     * @return the toggle state
     */
    public boolean toggleState() {
        return toggleState;
    }

    /**
     * The value as a String for telemetry
     *
     * @return the Button state
     */
    @NonNull
    public String toString() {
        return "" + isPressed;
    }
}