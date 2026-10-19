package org.firstinspires.ftc.teamcode.architecture.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** SDK {@link Gamepad} wrapped with per-control edge/cached suppliers. */
public final class LayerGamepad {
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

    public final EdgeBooleanSupplier touchpad;
    public final CachedDoubleSupplier touchpadFinger1X, touchpadFinger1Y;

    public LayerGamepad(Gamepad gamepad) {
        leftStickX  = doubleSupplier(() -> gamepad.left_stick_x);
        leftStickY  = doubleSupplier(() -> gamepad.left_stick_y);
        rightStickX = doubleSupplier(() -> gamepad.right_stick_x);
        rightStickY = doubleSupplier(() -> gamepad.right_stick_y);
        leftTrigger  = doubleSupplier(() -> gamepad.left_trigger);
        rightTrigger = doubleSupplier(() -> gamepad.right_trigger);

        a = boolSupplier(() -> gamepad.a);
        b = boolSupplier(() -> gamepad.b);
        x = boolSupplier(() -> gamepad.x);
        y = boolSupplier(() -> gamepad.y);

        dpadUp    = boolSupplier(() -> gamepad.dpad_up);
        dpadDown  = boolSupplier(() -> gamepad.dpad_down);
        dpadLeft  = boolSupplier(() -> gamepad.dpad_left);
        dpadRight = boolSupplier(() -> gamepad.dpad_right);

        leftBumper  = boolSupplier(() -> gamepad.left_bumper);
        rightBumper = boolSupplier(() -> gamepad.right_bumper);

        leftStickButton  = boolSupplier(() -> gamepad.left_stick_button);
        rightStickButton = boolSupplier(() -> gamepad.right_stick_button);

        guide = boolSupplier(() -> gamepad.guide);
        start = boolSupplier(() -> gamepad.start);
        back  = boolSupplier(() -> gamepad.back);

        touchpad         = boolSupplier(() -> gamepad.touchpad);
        touchpadFinger1X = doubleSupplier(() -> gamepad.touchpad_finger_1_x);
        touchpadFinger1Y = doubleSupplier(() -> gamepad.touchpad_finger_1_y);
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

    public void invalidateAll() {
        for (int i = 0; i < boolSuppliers.size(); i++) boolSuppliers.get(i).invalidate();
        for (int i = 0; i < doubleSuppliers.size(); i++) doubleSuppliers.get(i).invalidate();
    }
}
