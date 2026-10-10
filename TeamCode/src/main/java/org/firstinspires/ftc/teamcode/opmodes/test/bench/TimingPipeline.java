package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import org.opencv.core.Mat;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

/** Times each processFrame of the wrapped pipeline on the camera thread, into the sample set currently selected. */
final class TimingPipeline extends TimestampedOpenCvPipeline {
    private final TimestampedOpenCvPipeline inner;
    private volatile Samples.Synced into = Samples.synced();
    private volatile Samples.Synced captureAge = Samples.synced();
    private volatile long frames;

    TimingPipeline(TimestampedOpenCvPipeline inner) {
        this.inner = inner;
    }

    void recordInto(Samples.Synced samples, Samples.Synced captureAgeMs) {
        into = samples;
        captureAge = captureAgeMs;
    }

    long frames() {
        return frames;
    }

    @Override
    public void init(Mat firstFrame) {
        inner.init(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        long t0 = System.nanoTime();
        captureAge.add((t0 - captureTimeNanos) / 1e6);
        Mat out = inner.processFrame(input, captureTimeNanos);
        into.add((System.nanoTime() - t0) / 1e6);
        frames++;
        return out;
    }
}
