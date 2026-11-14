package org.firstinspires.ftc.teamcode.opmodes.test.bench;

/** LimelightBallSource's staleness rule replayed on recorded frame arrivals at other thresholds. */
final class StalenessReplay {
    private StalenessReplay() {}

    /** Times the source would have gone stale: once per gap between two arrivals longer than the threshold. */
    static int episodes(long[] arrivalsNs, double thresholdS) {
        long limitNs = Math.round(thresholdS * 1e9);
        int n = 0;
        for (int i = 1; i < arrivalsNs.length; i++) {
            if (arrivalsNs[i] - arrivalsNs[i - 1] > limitNs) n++;
        }
        return n;
    }

    static double longestGapMs(long[] arrivalsNs) {
        long longest = 0;
        for (int i = 1; i < arrivalsNs.length; i++) longest = Math.max(longest, arrivalsNs[i] - arrivalsNs[i - 1]);
        return longest / 1e6;
    }

    /** NaN when every threshold has an episode. */
    static double smallestClean(long[] arrivalsNs, double[] thresholdsS) {
        double best = Double.NaN;
        for (double t : thresholdsS) {
            if (episodes(arrivalsNs, t) == 0 && (Double.isNaN(best) || t < best)) best = t;
        }
        return best;
    }
}
