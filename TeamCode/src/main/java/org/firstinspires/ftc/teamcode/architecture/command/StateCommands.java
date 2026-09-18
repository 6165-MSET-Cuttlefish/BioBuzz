package org.firstinspires.ftc.teamcode.architecture.command;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.ConflictBehavior;

import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/** Ivy commands that drive {@link Module} state machines. */
public final class StateCommands {
    private StateCommands() {}

    /**
     * Instantly activates each state in order, requiring every owning Module.
     *
     * <p>Modules are resolved now, not at run time, so build these in {@code initialize()} — earlier
     * than that the State→Module bindings do not exist yet and this throws.
     */
    public static CommandBuilder set(State... states) {
        final State[] ordered = states.clone();
        Set<Object> requirements = new LinkedHashSet<>();
        for (State state : ordered) requirements.add(state.requireModule());
        return Command.build()
                .setStart(() -> {
                    for (State state : ordered) state.activate();
                })
                .setDone(() -> true)
                .requiring(requirements)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    /** Like {@link #set} but resolves the state when the command starts; the module must be named up front. */
    public static CommandBuilder setLazy(Module requirement, Supplier<? extends State> supplier) {
        return Command.build()
                .setStart(() -> supplier.get().activate())
                .setDone(() -> true)
                .requiring(requirement)
                .setConflictBehavior(ConflictBehavior.OVERRIDE);
    }
}
