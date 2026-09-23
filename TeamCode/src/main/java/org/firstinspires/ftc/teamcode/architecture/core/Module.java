package org.firstinspires.ftc.teamcode.architecture.core;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;

public abstract class Module {
    private static final int DEFAULT_HISTORY_SIZE = 50;
    private int maxHistorySize = DEFAULT_HISTORY_SIZE;

    private final List<State> states = new ArrayList<>();
    private final Map<Class<? extends State>, State> stateMap = new HashMap<>();
    private final List<TransitionGuard<?>> guards = new ArrayList<>();
    private final List<StateHook> enterHooks = new ArrayList<>();
    private final List<StateHook> exitHooks = new ArrayList<>();
    private final Deque<State> stateHistory = new ArrayDeque<>();
    private final List<Runnable> tunableRefreshers = new ArrayList<>();

    private final ElapsedTime stateTimer = new ElapsedTime();
    private DualTelemetry telemetry;
    private Command startupCommand;
    private boolean telemetryEnabled = true;
    private boolean writeEnabled = true;
    private String name;
    private String readSectionName;
    private String writeSectionName;

    public Module() {
        this.name = getClass().getSimpleName();
        recomputeSectionNames();
    }

    private void recomputeSectionNames() {
        this.readSectionName = "read." + this.name;
        this.writeSectionName = "write." + this.name;
    }

    public final String getReadSectionName() { return readSectionName; }
    public final String getWriteSectionName() { return writeSectionName; }

    protected abstract void initStates();
    protected abstract void read();
    protected abstract void write();

    public void init() {}

    /** Override to put hardware in a safe state on OpMode stop. */
    public void stop() {}

    protected void onStateChange() {}
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

    /** Render order in the MODULES telemetry section; lower first. */
    public int telemetryOrder() { return 0; }

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
                Class<?> enumClass = ((Enum<?>) s).getDeclaringClass();
                Object[] constants = enumClass.getEnumConstants();
                if (constants != null) {
                    for (Object c : constants) {
                        State variant = (State) c;
                        variant.setModule(this);
                        variant.resetValue();
                    }
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

    /** Transition to {@code newState}. True on success/no-op; false when the state class isn't registered or a guard rejected the transition. */
    public final boolean setState(State newState) {
        for (int i = 0; i < states.size(); i++) {
            State current = states.get(i);
            if (keyOf(newState) != keyOf(current)) continue;
            if (newState.equals(current)) return true;

            if (!checkGuards(current, newState)) return false;

            fireExitHooks(current);

            stateHistory.addLast(current);
            while (stateHistory.size() > maxHistorySize) stateHistory.pollFirst();

            states.set(i, newState);
            stateMap.put(keyOf(newState), newState);
            newState.setModule(this);
            stateTimer.reset();

            fireEnterHooks(newState);
            onStateChange();
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

    public final boolean isInAll(State... checkStates) {
        for (State check : checkStates) {
            State current = stateMap.get(keyOf(check));
            if (current == null || !current.equals(check)) return false;
        }
        return true;
    }

    public final long stateTimeMs() {
        return (long) stateTimer.milliseconds();
    }

    public final List<State> getHistory() {
        return Collections.unmodifiableList(new ArrayList<>(stateHistory));
    }

    public final void setMaxHistorySize(int size) {
        this.maxHistorySize = Math.max(1, size);
        while (stateHistory.size() > maxHistorySize) stateHistory.pollFirst();
    }

    public final int getMaxHistorySize() {
        return maxHistorySize;
    }

    protected final <T extends State> void guard(Class<T> stateClass, TransitionCheck<T> guard) {
        guards.add(new TransitionGuard<>(stateClass, guard));
    }

    protected final void onEnter(State state, Runnable action) {
        enterHooks.add(new StateHook(state, action));
    }

    protected final void onExit(State state, Runnable action) {
        exitHooks.add(new StateHook(state, action));
    }

    /** State-class identity key: enum constants with bodies report an anonymous subclass from getClass(), so every match path must use the declared type or setState between two body-bearing constants silently no-ops. */
    @SuppressWarnings("unchecked")
    private static Class<? extends State> keyOf(State s) {
        if (s instanceof Enum<?>) {
            return (Class<? extends State>) ((Enum<?>) s).getDeclaringClass();
        }
        return s.getClass();
    }

    @SuppressWarnings("unchecked")
    private <T extends State> boolean checkGuards(State from, State to) {
        Class<?> fromKey = keyOf(from);
        for (TransitionGuard<?> g : guards) {
            if (g.stateClass == fromKey) {
                TransitionGuard<T> typedGuard = (TransitionGuard<T>) g;
                if (!typedGuard.check.test((T) from, (T) to)) return false;
            }
        }
        return true;
    }

    private void fireEnterHooks(State state) {
        for (StateHook h : enterHooks) {
            if (h.state.equals(state)) h.action.run();
        }
    }

    private void fireExitHooks(State state) {
        for (StateHook h : exitHooks) {
            if (h.state.equals(state)) h.action.run();
        }
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

    public final Module withName(String name) {
        this.name = name;
        recomputeSectionNames();
        return this;
    }

    public final String getName() { return name; }

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

    @FunctionalInterface
    public interface TransitionCheck<T extends State> {
        boolean test(T from, T to);
    }

    private static class TransitionGuard<T extends State> {
        final Class<T> stateClass;
        final TransitionCheck<T> check;

        TransitionGuard(Class<T> stateClass, TransitionCheck<T> check) {
            this.stateClass = stateClass;
            this.check = check;
        }
    }

    private static class StateHook {
        final State state;
        final Runnable action;

        StateHook(State state, Runnable action) {
            this.state = state;
            this.action = action;
        }
    }
}
