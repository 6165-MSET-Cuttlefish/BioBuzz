package org.firstinspires.ftc.teamcode.architecture.command;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;

import java.util.LinkedHashSet;
import java.util.Set;

import org.firstinspires.ftc.teamcode.architecture.core.State;

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
}
