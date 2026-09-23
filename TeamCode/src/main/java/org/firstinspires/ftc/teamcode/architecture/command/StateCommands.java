package org.firstinspires.ftc.teamcode.architecture.command;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;

import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/** Ivy commands that drive {@link Module} state machines. */
public final class StateCommands {
    private StateCommands() {}

    public static CommandBuilder set(State... states) {
        final State[] ordered = states.clone();
        Set<Object> requirements = new LinkedHashSet<>();
        for (State state : ordered) requirements.add(state.requireModule());
        return Commands.instant(() -> {
            for (State state : ordered) state.activate();
        }).requiring(requirements);
    }

    /** Like {@link #set} but resolves the state when the command starts; the module must be named up front. */
    public static CommandBuilder setLazy(Module requirement, Supplier<? extends State> supplier) {
        return Commands.instant(() -> supplier.get().activate()).requiring(requirement);
    }
}
