package org.firstinspires.ftc.teamcode.architecture.input;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

/**
 * Gamepad façade delegating to the active {@link LayerGamepad} on a {@link LayerStack}; reads
 * neutral when no layer is active or a layer switch is in progress.
 */
public class LayeredGamepad<T> {
    private final LayerStack<T> layerStack;
    // Frame-scoped, not a boolean: suppression auto-expires next loop even if invalidateAll() is never pumped.
    private long suppressUntilFrame = Long.MIN_VALUE;
    private final List<EdgeBooleanSupplier> trackedBools = new ArrayList<>();
    private final List<CachedDoubleSupplier> trackedDoubles = new ArrayList<>();

    private final CachedDoubleSupplier leftStickX, leftStickY;
    private final CachedDoubleSupplier rightStickX, rightStickY;
    private final CachedDoubleSupplier leftTrigger, rightTrigger;
    private final EdgeBooleanSupplier a, b, x, y;
    private final EdgeBooleanSupplier dpadUp, dpadDown, dpadLeft, dpadRight;
    private final EdgeBooleanSupplier leftBumper, rightBumper;
    private final EdgeBooleanSupplier start, back, guide;
    private final EdgeBooleanSupplier leftStickButton, rightStickButton;
    private final EdgeBooleanSupplier touchpad;
    private final CachedDoubleSupplier touchpadFinger1X, touchpadFinger1Y;

    public LayeredGamepad(LayerStack<T> layerStack) {
        if (layerStack == null) throw new IllegalArgumentException("layerStack cannot be null");
        this.layerStack = layerStack;
        layerStack.setOnLayerChange(this::onLayerChanged);

        leftStickX  = mappedDouble(cg -> cg.leftStickX);
        leftStickY  = mappedDouble(cg -> cg.leftStickY);
        rightStickX = mappedDouble(cg -> cg.rightStickX);
        rightStickY = mappedDouble(cg -> cg.rightStickY);
        leftTrigger  = mappedDouble(cg -> cg.leftTrigger);
        rightTrigger = mappedDouble(cg -> cg.rightTrigger);

        a = mappedBool(cg -> cg.a);
        b = mappedBool(cg -> cg.b);
        x = mappedBool(cg -> cg.x);
        y = mappedBool(cg -> cg.y);

        dpadUp    = mappedBool(cg -> cg.dpadUp);
        dpadDown  = mappedBool(cg -> cg.dpadDown);
        dpadLeft  = mappedBool(cg -> cg.dpadLeft);
        dpadRight = mappedBool(cg -> cg.dpadRight);

        leftBumper  = mappedBool(cg -> cg.leftBumper);
        rightBumper = mappedBool(cg -> cg.rightBumper);

        start = mappedBool(cg -> cg.start);
        back  = mappedBool(cg -> cg.back);
        guide = mappedBool(cg -> cg.guide);

        leftStickButton  = mappedBool(cg -> cg.leftStickButton);
        rightStickButton = mappedBool(cg -> cg.rightStickButton);

        touchpad         = mappedBool(cg -> cg.touchpad);
        touchpadFinger1X = mappedDouble(cg -> cg.touchpadFinger1X);
        touchpadFinger1Y = mappedDouble(cg -> cg.touchpadFinger1Y);
    }

    private EdgeBooleanSupplier mappedBool(Function<LayerGamepad, EdgeBooleanSupplier> getter) {
        BooleanSupplier source = () -> {
            LayerGamepad active = getActiveGamepad();
            return active != null && getter.apply(active).getValue();
        };
        return new EdgeBooleanSupplier(source);
    }

    private CachedDoubleSupplier mappedDouble(Function<LayerGamepad, CachedDoubleSupplier> getter) {
        DoubleSupplier source = () -> {
            LayerGamepad active = getActiveGamepad();
            return active != null ? getter.apply(active).getValue() : 0.0;
        };
        return new CachedDoubleSupplier(source);
    }

    public LayerStack<T> getLayerStack() { return layerStack; }

    public LayerGamepad getActiveGamepad() {
        if (InputClock.current() < suppressUntilFrame) return null;
        return layerStack.getGamepad();
    }

    public T getCurrentLayer() { return layerStack.getLayer(); }

    public void setLayer(T layer) {
        if (layer == null) throw new IllegalArgumentException("layer cannot be null");
        if (layer.equals(layerStack.getLayer())) return;
        // The stack fires onLayerChanged() once it commits the change — that primes+suppresses.
        layerStack.setLayer(layer);
    }

    private void onLayerChanged() {
        // Prime BEFORE arming suppression, so priming reads the real new-layer state.
        layerStack.invalidateAll();
        primeAllSuppliers();
        suppressUntilFrame = InputClock.current() + 1;
    }

    public boolean isActive(LayerGamepad gamepad) {
        return layerStack.isActive(gamepad);
    }

    public void update() {
        layerStack.update();
    }

    /** Optional per-loop eager refresh; suppliers otherwise refresh lazily off {@link InputClock} when queried. */
    public void invalidateAll() {
        // Underlying gamepads first, or the wrappers stay one frame behind.
        layerStack.invalidateAll();
        forEachSupplier(EdgeBooleanSupplier::invalidate, CachedDoubleSupplier::invalidate);
    }

    public void invalidateActive() {
        layerStack.invalidateActive();
        forEachSupplier(EdgeBooleanSupplier::invalidate, CachedDoubleSupplier::invalidate);
    }

    /**
     * Register derived suppliers (e.g. {@code gamepad.LT().greaterThan(0.5)}) so a layer switch primes
     * them; an unprimed derived supplier fires a spurious edge on the first layer change.
     */
    public LayeredGamepad<T> track(EdgeBooleanSupplier... suppliers) {
        for (EdgeBooleanSupplier s : suppliers) if (s != null) trackedBools.add(s);
        return this;
    }

    public LayeredGamepad<T> trackDouble(CachedDoubleSupplier... suppliers) {
        for (CachedDoubleSupplier s : suppliers) if (s != null) trackedDoubles.add(s);
        return this;
    }

    private void primeAllSuppliers() {
        forEachSupplier(EdgeBooleanSupplier::primeToCurrentState,
                CachedDoubleSupplier::primeToCurrentState);
    }

    private void forEachSupplier(java.util.function.Consumer<EdgeBooleanSupplier> bool,
                                 java.util.function.Consumer<CachedDoubleSupplier> dbl) {
        bool.accept(a); bool.accept(b); bool.accept(x); bool.accept(y);
        bool.accept(dpadUp); bool.accept(dpadDown); bool.accept(dpadLeft); bool.accept(dpadRight);
        bool.accept(leftBumper); bool.accept(rightBumper);
        bool.accept(start); bool.accept(back); bool.accept(guide);
        bool.accept(leftStickButton); bool.accept(rightStickButton);
        bool.accept(touchpad);

        dbl.accept(leftStickX); dbl.accept(leftStickY);
        dbl.accept(rightStickX); dbl.accept(rightStickY);
        dbl.accept(leftTrigger); dbl.accept(rightTrigger);
        dbl.accept(touchpadFinger1X); dbl.accept(touchpadFinger1Y);

        for (int i = 0; i < trackedBools.size(); i++) bool.accept(trackedBools.get(i));
        for (int i = 0; i < trackedDoubles.size(); i++) dbl.accept(trackedDoubles.get(i));
    }

    public CachedDoubleSupplier getLeftStickX()  { return leftStickX; }
    public CachedDoubleSupplier getLeftStickY()  { return leftStickY; }
    public CachedDoubleSupplier getRightStickX() { return rightStickX; }
    public CachedDoubleSupplier getRightStickY() { return rightStickY; }
    public CachedDoubleSupplier LT() { return leftTrigger; }
    public CachedDoubleSupplier RT() { return rightTrigger; }

    public EdgeBooleanSupplier A() { return a; }
    public EdgeBooleanSupplier B() { return b; }
    public EdgeBooleanSupplier X() { return x; }
    public EdgeBooleanSupplier Y() { return y; }

    public EdgeBooleanSupplier DPAD_UP()    { return dpadUp; }
    public EdgeBooleanSupplier DPAD_DOWN()  { return dpadDown; }
    public EdgeBooleanSupplier DPAD_LEFT()  { return dpadLeft; }
    public EdgeBooleanSupplier DPAD_RIGHT() { return dpadRight; }

    public EdgeBooleanSupplier LB() { return leftBumper; }
    public EdgeBooleanSupplier RB() { return rightBumper; }

    public EdgeBooleanSupplier getStart() { return start; }
    public EdgeBooleanSupplier getBack()  { return back; }
    public EdgeBooleanSupplier getGuide() { return guide; }

    public EdgeBooleanSupplier LSB() { return leftStickButton; }
    public EdgeBooleanSupplier RSB() { return rightStickButton; }

    public EdgeBooleanSupplier getTouchpad() { return touchpad; }
    public CachedDoubleSupplier  TX() { return touchpadFinger1X; }
    public CachedDoubleSupplier  TY() { return touchpadFinger1Y; }

    public EdgeBooleanSupplier C() { return A(); }
    public EdgeBooleanSupplier O() { return B(); }
    public EdgeBooleanSupplier Q() { return X(); }
    public EdgeBooleanSupplier T() { return Y(); }
    public EdgeBooleanSupplier getOptions() { return getStart(); }

    @Override
    public String toString() {
        return "LayeredGamepad{currentLayer=" + getCurrentLayer() + "}";
    }
}
