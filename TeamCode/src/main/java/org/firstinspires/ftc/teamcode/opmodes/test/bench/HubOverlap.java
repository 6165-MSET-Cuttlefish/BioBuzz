package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/** The write sets Bench: Photon's two-hubs-at-once phase times, named by Control Hub + Expansion Hub writes. */
final class HubOverlap {
    static final int LOOPS = 300;
    static final long SEED = 6165;

    private HubOverlap() {}

    static <T> Map<String, List<T>> sets(List<T> chMotors, List<T> chServos, List<T> ehMotors, List<T> ehServos) {
        List<T> ch = mixed(chMotors, chServos, chMotors.size() + chServos.size());
        List<T> eh = mixed(ehMotors, ehServos, ehMotors.size() + ehServos.size());
        if (ch.size() != 6 || eh.size() != 6) {
            throw new IllegalArgumentException("the sets assume 6 channels per hub, got " + ch.size() + " + " + eh.size());
        }
        Map<String, List<T>> sets = new LinkedHashMap<>();
        sets.put("6 + 0", new ArrayList<>(ch));
        sets.put("0 + 6", new ArrayList<>(eh));
        sets.put("6 + 6", concat(ch, eh));
        sets.put("6 + 6 alternating", alternate(ch, eh));
        sets.put("3 + 3", concat(ch.subList(0, 3), eh.subList(0, 3)));
        sets.put("5 + 1", concat(ch.subList(0, 5), eh.subList(0, 1)));
        sets.put("1 + 5", concat(ch.subList(0, 1), eh.subList(0, 5)));
        return sets;
    }

    /** The first n of a hub's channels, a motor and a servo in turn while both last, so a partial set mixes kinds. */
    static <T> List<T> mixed(List<T> motors, List<T> servos, int n) {
        List<T> out = new ArrayList<>();
        int m = 0;
        int s = 0;
        while (out.size() < n && (m < motors.size() || s < servos.size())) {
            boolean motorTurn = out.size() % 2 == 0;
            if ((motorTurn && m < motors.size()) || s >= servos.size()) out.add(motors.get(m++));
            else out.add(servos.get(s++));
        }
        return out;
    }

    static <T> List<T> alternate(List<T> first, List<T> second) {
        List<T> out = new ArrayList<>();
        for (int i = 0; i < Math.max(first.size(), second.size()); i++) {
            if (i < first.size()) out.add(first.get(i));
            if (i < second.size()) out.add(second.get(i));
        }
        return out;
    }

    static <T> List<T> concat(List<T> first, List<T> second) {
        List<T> out = new ArrayList<>(first);
        out.addAll(second);
        return out;
    }

    /** Every set once per round, shuffled each round so a slow drift or the previous set favours none. */
    static int[] roundOrder(int sets, Random random) {
        int[] order = new int[sets];
        for (int i = 0; i < sets; i++) order[i] = i;
        for (int i = sets - 1; i > 0; i--) {
            int j = random.nextInt(i + 1);
            int t = order[i];
            order[i] = order[j];
            order[j] = t;
        }
        return order;
    }
}
