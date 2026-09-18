package org.firstinspires.ftc.teamcode.architecture.core;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * State-machine state. Implement on an enum and set the setpoint via {@link #setValue(double)} in the constructor.
 *
 * <p>Backing maps are static so values survive OpMode reconstruction (Sloth hot-reload, re-runs); {@link #clearModuleBindings()} drops stale State→Module pointers at init.
 */
public interface State {

    Map<State, Double> VALUES = new ConcurrentHashMap<>();
    Map<State, Double> INITIAL_VALUES = new ConcurrentHashMap<>();
    Map<State, Module> MODULES = new ConcurrentHashMap<>();

    static void clearModuleBindings() {
        MODULES.clear();
    }

    default double getValue() {
        Double v = VALUES.get(this);
        return v == null ? 0.0 : v;
    }

    default void setValue(double value) {
        Double existing = VALUES.get(this);
        if (existing != null && existing == value) return;
        INITIAL_VALUES.putIfAbsent(this, value);
        VALUES.put(this, value);
    }

    default void resetValue() {
        Double initial = INITIAL_VALUES.get(this);
        if (initial != null) VALUES.put(this, initial);
    }

    default Module getModule() {
        return MODULES.get(this);
    }

    default void setModule(Module module) {
        MODULES.put(this, module);
    }

    /** Transition this state's owning module to this state. False if unbound or guard-rejected. */
    default boolean activate() {
        Module m = getModule();
        if (m == null) {
            System.err.println("[State] activate() called on " + this + " but no module is registered");
            return false;
        }
        return m.setState(this);
    }

    default Module requireModule() {
        Module m = getModule();
        if (m == null) {
            throw new IllegalStateException(
                    "State " + this + " has no registered module. "
                    + "Call setStates() with this state during Module.initStates().");
        }
        return m;
    }
}
