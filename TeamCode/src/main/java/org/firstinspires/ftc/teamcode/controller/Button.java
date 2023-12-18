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
    public Button() {
        isPressed = false;
        wasPressed = false;
        toggleState = false;
    }

    /**
     * Updates the Button with the new state
     *
     * @param isNowPressed whether the button is pressed
     */
    public void update(boolean isNowPressed) {
        wasPressed = isPressed;
        isPressed = isNowPressed;
        toggleState = (isPressed && !wasPressed) != toggleState;
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
     * The Button's toggle state switches when the Button is pressed
     *
     * @return the toggle state
     */
    public boolean getToggleState() {
        return toggleState;
    }

    /**
     * The value as a String for telemetry
     *
     * @return the Button state
     */
    @NonNull
    public String toString() {
        return String.valueOf(isPressed);
    }
}
