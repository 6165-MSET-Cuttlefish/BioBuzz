package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import java.util.ArrayList;
import java.util.List;

final class OctoQuadMath {
    static final int CHANNELS = 8;

    private OctoQuadMath() {}

    static List<int[]> diff(int[] before, int[] after) {
        if (before.length != CHANNELS || after.length != CHANNELS) {
            throw new IllegalArgumentException("an OctoQuad table has " + CHANNELS + " channels, got " + before.length
                    + " and " + after.length);
        }
        List<int[]> out = new ArrayList<>();
        for (int ch = 0; ch < CHANNELS; ch++) {
            if (before[ch] != after[ch]) out.add(new int[] {ch, before[ch], after[ch]});
        }
        return out;
    }

    static boolean kept(int[] table, int channel, int set) {
        return table[channel] == set;
    }

    static boolean zeroed(int[] positions, int[] channels, int tolerance) {
        for (int ch : channels) {
            if (Math.abs(positions[ch]) > tolerance) return false;
        }
        return true;
    }

    static boolean moved(int[] before, int[] after, int[] channels, int tolerance) {
        for (int ch : channels) {
            if (Math.abs((long) after[ch] - before[ch]) > tolerance) return true;
        }
        return false;
    }

    static int[] directionBits(boolean[] reversed) {
        int[] bits = new int[reversed.length];
        for (int i = 0; i < reversed.length; i++) bits[i] = reversed[i] ? 1 : 0;
        return bits;
    }
}
