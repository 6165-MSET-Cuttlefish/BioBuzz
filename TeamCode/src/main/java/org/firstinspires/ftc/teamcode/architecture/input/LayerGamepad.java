package org.firstinspires.ftc.teamcode.architecture.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * SDK {@link Gamepad} wrapped with per-control edge/cached suppliers. While {@link #atRest}
 * is true, every supplier reads neutral so inactive layers are physically present but silent.
 */
public final class LayerGamepad {
    private final Gamepad gamepad;
    private boolean atRest = false;

    private final List<EdgeBooleanSupplier> boolSuppliers = new ArrayList<>();
    private final List<CachedDoubleSupplier> doubleSuppliers = new ArrayList<>();

    public final CachedDoubleSupplier leftStickX, leftStickY;
    public final CachedDoubleSupplier rightStickX, rightStickY;
    public final CachedDoubleSupplier leftTrigger, rightTrigger;

    public final EdgeBooleanSupplier a, b, x, y;
    public final EdgeBooleanSupplier dpadUp, dpadDown, dpadLeft, dpadRight;
    public final EdgeBooleanSupplier leftBumper, rightBumper;
    public final EdgeBooleanSupplier leftStickButton, rightStickButton;
    public final EdgeBooleanSupplier guide, start, back;

    public final EdgeBooleanSupplier touchpad, touchpadFinger1, touchpadFinger2;
    public final CachedDoubleSupplier touchpadFinger1X, touchpadFinger1Y;
    public final CachedDoubleSupplier touchpadFinger2X, touchpadFinger2Y;

    public LayerGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;

        leftStickX  = doubleSupplier(() -> atRest ? 0.0 : gamepad.left_stick_x);
        leftStickY  = doubleSupplier(() -> atRest ? 0.0 : gamepad.left_stick_y);
        rightStickX = doubleSupplier(() -> atRest ? 0.0 : gamepad.right_stick_x);
        rightStickY = doubleSupplier(() -> atRest ? 0.0 : gamepad.right_stick_y);
        leftTrigger  = doubleSupplier(() -> atRest ? 0.0 : gamepad.left_trigger);
        rightTrigger = doubleSupplier(() -> atRest ? 0.0 : gamepad.right_trigger);

        a = boolSupplier(() -> !atRest && gamepad.a);
        b = boolSupplier(() -> !atRest && gamepad.b);
        x = boolSupplier(() -> !atRest && gamepad.x);
        y = boolSupplier(() -> !atRest && gamepad.y);

        dpadUp    = boolSupplier(() -> !atRest && gamepad.dpad_up);
        dpadDown  = boolSupplier(() -> !atRest && gamepad.dpad_down);
        dpadLeft  = boolSupplier(() -> !atRest && gamepad.dpad_left);
        dpadRight = boolSupplier(() -> !atRest && gamepad.dpad_right);

        leftBumper  = boolSupplier(() -> !atRest && gamepad.left_bumper);
        rightBumper = boolSupplier(() -> !atRest && gamepad.right_bumper);

        leftStickButton  = boolSupplier(() -> !atRest && gamepad.left_stick_button);
        rightStickButton = boolSupplier(() -> !atRest && gamepad.right_stick_button);

        guide = boolSupplier(() -> !atRest && gamepad.guide);
        start = boolSupplier(() -> !atRest && gamepad.start);
        back  = boolSupplier(() -> !atRest && gamepad.back);

        touchpad         = boolSupplier(() -> !atRest && gamepad.touchpad);
        touchpadFinger1  = boolSupplier(() -> !atRest && gamepad.touchpad_finger_1);
        touchpadFinger2  = boolSupplier(() -> !atRest && gamepad.touchpad_finger_2);
        touchpadFinger1X = doubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_1_x);
        touchpadFinger1Y = doubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_1_y);
        touchpadFinger2X = doubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_2_x);
        touchpadFinger2Y = doubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_2_y);
    }

    private EdgeBooleanSupplier boolSupplier(BooleanSupplier source) {
        EdgeBooleanSupplier s = new EdgeBooleanSupplier(source);
        boolSuppliers.add(s);
        return s;
    }

    private CachedDoubleSupplier doubleSupplier(DoubleSupplier source) {
        CachedDoubleSupplier s = new CachedDoubleSupplier(source);
        doubleSuppliers.add(s);
        return s;
    }

    public void setAtRest(boolean atRest) {
        boolean wasAtRest = this.atRest;
        this.atRest = atRest;
        // On rest→active: prime so a button held at activation doesn't fire wasJustPressed.
        if (wasAtRest && !atRest) primeAllSuppliers();
    }

    public boolean isAtRest() {
        return atRest;
    }

    public void invalidateAll() {
        for (int i = 0; i < boolSuppliers.size(); i++) boolSuppliers.get(i).invalidate();
        for (int i = 0; i < doubleSuppliers.size(); i++) doubleSuppliers.get(i).invalidate();
    }

    private void primeAllSuppliers() {
        for (int i = 0; i < boolSuppliers.size(); i++) boolSuppliers.get(i).primeToCurrentState();
        for (int i = 0; i < doubleSuppliers.size(); i++) doubleSuppliers.get(i).primeToCurrentState();
    }

    /** Escape hatch for SDK fields not wrapped here. */
    public Gamepad getRawGamepad() {
        return gamepad;
    }
}
