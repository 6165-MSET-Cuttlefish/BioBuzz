package org.firstinspires.ftc.teamcode.architecture.core;

import com.pedropathing.ivy.Command;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;

public abstract class Module {
    private final List<State> states = new ArrayList<>();
    private final Map<Class<? extends State>, State> stateMap = new HashMap<>();
    private final List<Runnable> tunableRefreshers = new ArrayList<>();

    private DualTelemetry telemetry;
    private Command startupCommand;
    private boolean telemetryEnabled = true;
    private boolean writeEnabled = true;
    private final String name;
    private final String readSectionName;
    private final String writeSectionName;

    public Module() {
        this.name = getClass().getSimpleName();
        this.readSectionName = "read." + this.name;
        this.writeSectionName = "write." + this.name;
    }

    public final String getReadSectionName() { return readSectionName; }
    public final String getWriteSectionName() { return writeSectionName; }

    protected abstract void initStates();
    protected abstract void read();
    protected abstract void write();

    public void init() {}

    /** Commands every output directly to a safe state at OpMode stop; it can also run mid-OpMode, with write() resuming after. */
    public abstract void stop();

    protected void onTelemetry() {}

    /** Wire a State's setpoint to a live source; refreshed once per loop before {@link #read()}. Call after {@link #setStates(State...)}. */
    protected final void bindTunable(State state, DoubleSupplier supplier) {
        Runnable refresh = () -> state.setValue(supplier.getAsDouble());
        refresh.run();
        tunableRefreshers.add(refresh);
    }

    final void refreshTunables() {
        for (int i = 0; i < tunableRefreshers.size(); i++) {
            tunableRefreshers.get(i).run();
        }
    }

    /** Register the initial state per state-class. For enums, every variant is bound to this module and reset to its initial value so a re-run starts clean. */
    protected final void setStates(State... initialStates) {
        states.clear();
        stateMap.clear();
        for (State s : initialStates) {
            s.setModule(this);
            states.add(s);
            stateMap.put(keyOf(s), s);

            if (s instanceof Enum<?>) {
                // getDeclaringClass handles enum constants with bodies (anonymous subclasses whose own getEnumConstants() returns null).
                for (Object c : ((Enum<?>) s).getDeclaringClass().getEnumConstants()) {
                    State variant = (State) c;
                    variant.setModule(this);
                    variant.resetValue();
                }
            }
        }
    }

    /** O(1) map lookup, with a list-scan fallback to preserve isInstance() semantics. */
    @SuppressWarnings("unchecked")
    public final <T extends State> T getState(Class<T> stateClass) {
        State s = stateMap.get(stateClass);
        if (s != null) return (T) s;
        for (int i = 0; i < states.size(); i++) {
            State candidate = states.get(i);
            if (stateClass.isInstance(candidate)) return (T) candidate;
        }
        throw new IllegalArgumentException("No state of type: " + stateClass.getSimpleName());
    }

    /** Transition to {@code newState}. True on success/no-op; false when the state class isn't registered. */
    public final boolean setState(State newState) {
        for (int i = 0; i < states.size(); i++) {
            State current = states.get(i);
            if (keyOf(newState) != keyOf(current)) continue;
            if (newState.equals(current)) return true;

            states.set(i, newState);
            stateMap.put(keyOf(newState), newState);
            newState.setModule(this);
            return true;
        }
        return false;
    }

    public final boolean isInAny(State... checkStates) {
        for (State check : checkStates) {
            // keyOf, not getState(check.getClass()): the latter throws for enum constants with bodies.
            State current = stateMap.get(keyOf(check));
            if (current != null && current.equals(check)) return true;
        }
        return false;
    }

    /** State-class identity key: enum constants with bodies report an anonymous subclass from getClass(), so every match path must use the declared type or setState between two body-bearing constants silently no-ops. */
    @SuppressWarnings("unchecked")
    private static Class<? extends State> keyOf(State s) {
        if (s instanceof Enum<?>) {
            return (Class<? extends State>) ((Enum<?>) s).getDeclaringClass();
        }
        return s.getClass();
    }

    final void setTelemetry(DualTelemetry t) { this.telemetry = t; }

    protected final DualTelemetry getTelemetry() { return telemetry; }

    protected final String getStateString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < states.size(); i++) {
            if (i > 0) sb.append(" | ");
            sb.append(states.get(i));
        }
        return sb.toString();
    }

    protected final void telemetry() {
        if (!telemetryEnabled) return;
        telemetry.addModuleHeader(name, getStateString());
        onTelemetry();
    }

    public final void setTelemetryEnabled(boolean enabled) { this.telemetryEnabled = enabled; }

    public final void setWriteEnabled(boolean enabled) { this.writeEnabled = enabled; }
    public final boolean isWriteEnabled() { return writeEnabled; }

    /** Command scheduled once at start(). NOT re-armed later. */
    public final void setStartupCommand(Command command) { this.startupCommand = command; }
    public final Command getStartupCommand() { return startupCommand; }

    protected final void logDS(String caption, Object value) {
        telemetry.addDSData(name + " " + caption, value);
    }

    protected final void logDS(String caption, String format, Object... args) {
        telemetry.addDSData(name + " " + caption, format, args);
    }

    protected final void logDashboard(String caption, Object value) {
        telemetry.addDashboardData(name + " " + caption, value);
    }

    protected final void logDashboard(String caption, String format, Object... args) {
        telemetry.addDashboardData(name + " " + caption, format, args);
    }

    protected final void log(String caption, Object value) {
        if (telemetryEnabled) telemetry.addData(name + " " + caption, value);
    }

    protected final void log(String caption, String format, Object... args) {
        if (telemetryEnabled) telemetry.addData(name + " " + caption, String.format(format, args));
    }
}
