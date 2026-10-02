package org.firstinspires.ftc.teamcode.modules.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/**
 * Puts limelight/ball_pipeline.vpr and ball_contour_snapscript.py on a Limelight pipeline and switches to it, on one
 * background thread, since Limelight3A's POSTs block for up to 15 s. One request runs at a time; a newer request
 * replaces one that hasn't started. The uploaded script reports {@link #STAMP} instead of its file's SCRIPT_ID, which
 * proves the upload is the script running.
 */
public final class LimelightSync {
    /** What the uploaded copy reports in llpython[0]: a fingerprint of both files, below 2^24. */
    public static final double STAMP = LimelightFiles.BALL_SCRIPT_STAMP;
    /** What a copy pasted into the Limelight's web editor reports. */
    public static final double FILE_SCRIPT_ID = LimelightFiles.BALL_SCRIPT_ID;

    private static final String PYTHON_PIPELINE_TYPE = "pipe_python";
    private static final int ATTEMPTS = 3;
    private static final long SWITCH_WAIT_MS = 2000;
    // The Limelight loads a pipeline's script when it switches to that pipeline. An upload to the pipeline it is
    // running, or sent within about a second of leaving it, keeps the old script (measured on a Limelight 3A).
    private static final long PARK_SETTLE_MS = 1500;
    private static final long LOAD_WAIT_MS = 3000;
    private static final long POLL_MS = 20;

    public static final class Request {
        public final int pipeline;
        /** False only switches, for a script someone is tuning in the web editor. */
        public final boolean upload;
        private final Limelight3A limelight;
        private volatile boolean cancelled;
        private volatile boolean done;
        private volatile String problem;
        private volatile Throwable crash;

        Request(Limelight3A limelight, int pipeline, boolean upload) {
            this.limelight = limelight;
            this.pipeline = pipeline;
            this.upload = upload;
        }

        public boolean isDone() { return done; }

        /** Null while running and after success. */
        public String problem() { return problem; }

        /** Stops it before its next POST; one already sent still lands. */
        public void cancel() { cancelled = true; }

        /** A bug in the sync thread surfaces on the caller's thread, which the SDK reports, instead of killing the app. */
        public void rethrowIfCrashed() {
            if (crash != null) throw new IllegalStateException("LimelightSync crashed", crash);
        }

        private void finish(String found) {
            problem = found;
            done = true;
        }
    }

    private static final class Cancelled extends Exception {}

    private static final Object lock = new Object();
    private static Request pending;
    private static boolean running;

    private LimelightSync() {}

    public static Request request(Limelight3A limelight, int pipeline, boolean upload) {
        Request request = new Request(limelight, pipeline, upload);
        synchronized (lock) {
            if (pending != null) {
                pending.finish(String.format("replaced by a request for pipeline %d", pipeline));
            }
            pending = request;
            if (!running) {
                running = true;
                Thread worker = new Thread(LimelightSync::drain, "LimelightSync");
                worker.setDaemon(true);
                worker.start();
            }
        }
        return request;
    }

    private static void drain() {
        while (true) {
            Request request;
            synchronized (lock) {
                request = pending;
                pending = null;
                if (request == null) {
                    running = false;
                    return;
                }
            }
            try {
                request.finish(run(request));
            } catch (Cancelled e) {
                request.finish("cancelled");
            } catch (RuntimeException | Error e) {
                request.crash = e;
                request.finish("crashed: " + e);
            } catch (InterruptedException e) {
                request.finish("interrupted");
                synchronized (lock) {
                    if (pending != null) pending.finish("interrupted");
                    pending = null;
                    running = false;
                }
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    private static String run(Request r) throws InterruptedException, Cancelled {
        Limelight3A ll = r.limelight;
        int target = r.pipeline;
        if (!r.upload) {
            check(r);
            if (!ll.pipelineSwitch(target)) return String.format("pipeline switch to %d rejected", target);
            return pythonProblem(r, target);
        }

        int park = target == 0 ? 1 : 0;
        int uploads = 0;
        for (int attempt = 0; attempt < ATTEMPTS; attempt++) {
            check(r);
            if (!ll.pipelineSwitch(park)) {
                return String.format("pipeline switch to %d (parking before the upload to %d) rejected", park, target);
            }
            if (!waitForPipeline(r, park, System.currentTimeMillis())) continue;
            sleep(r, PARK_SETTLE_MS);
            check(r);
            if (!ll.uploadPipeline(LimelightFiles.BALL_PIPELINE, target)) {
                return String.format("pipeline upload to %d rejected", target);
            }
            check(r);
            if (!ll.uploadPython(LimelightFiles.BALL_SCRIPT, target)) {
                return String.format("script upload to %d rejected", target);
            }
            uploads++;
            check(r);
            if (!ll.pipelineSwitch(target)) return String.format("pipeline switch to %d rejected", target);
            if (waitForStamp(r, target, System.currentTimeMillis())) return pythonProblem(r, target);
        }
        if (uploads == 0) return String.format("the Limelight never reported park pipeline %d", park);
        return String.format("pipeline %d never reported the upload's stamp %d after %d upload%s (llpython[0] %s); check "
                        + "the script for errors in the Limelight's web UI, or power-cycle the Limelight",
                target, (long) STAMP, uploads, uploads == 1 ? "" : "s",
                LimelightBallSource.number(ll.getLatestResult().getPythonOutput()[0]));
    }

    private static void check(Request r) throws Cancelled {
        if (r.cancelled) throw new Cancelled();
    }

    private static void sleep(Request r, long ms) throws InterruptedException, Cancelled {
        long deadline = System.nanoTime() + ms * 1_000_000L;
        while (System.nanoTime() < deadline) {
            check(r);
            Thread.sleep(POLL_MS);
        }
    }

    // getControlHubTimeStamp() is the wall clock when the SDK parsed the result.
    private static boolean waitForPipeline(Request r, int pipeline, long fromMillis)
            throws InterruptedException, Cancelled {
        long deadline = System.nanoTime() + SWITCH_WAIT_MS * 1_000_000L;
        while (System.nanoTime() < deadline) {
            check(r);
            LLResult result = r.limelight.getLatestResult();
            if (result != null && result.getControlHubTimeStamp() > fromMillis
                    && result.getPipelineIndex() == pipeline) {
                return true;
            }
            Thread.sleep(POLL_MS);
        }
        return false;
    }

    private static boolean waitForStamp(Request r, int pipeline, long fromMillis)
            throws InterruptedException, Cancelled {
        long deadline = System.nanoTime() + LOAD_WAIT_MS * 1_000_000L;
        while (System.nanoTime() < deadline) {
            check(r);
            LLResult result = r.limelight.getLatestResult();
            if (result != null && result.getControlHubTimeStamp() > fromMillis
                    && result.getPipelineIndex() == pipeline && result.getPythonOutput()[0] == STAMP) {
                return true;
            }
            Thread.sleep(POLL_MS);
        }
        return false;
    }

    // llpython outlives its pipeline, so a non-Python pipeline still shows the last script's output. LLResult's
    // getPipelineType() is always empty (the SDK reads "pipelineType", results send "pTYPE"); /status has it.
    private static String pythonProblem(Request r, int pipeline) throws InterruptedException, Cancelled {
        long deadline = System.nanoTime() + SWITCH_WAIT_MS * 1_000_000L;
        while (System.nanoTime() < deadline) {
            check(r);
            LLStatus status = r.limelight.getStatus();
            String type = status.getPipelineType();
            if (status.getPipelineIndex() == pipeline && type != null && !type.isEmpty()) {
                return PYTHON_PIPELINE_TYPE.equals(type) ? null
                        : String.format("pipeline %d is a %s pipeline, not Python", pipeline, type);
            }
            Thread.sleep(POLL_MS);
        }
        return String.format("the Limelight's status never showed pipeline %d", pipeline);
    }

    /** A stamp from any build, which an upload's copy edited in the web editor still reports. */
    static boolean isStamp(double id) {
        return id >= 0x800000 && id < 0x1000000 && id == Math.rint(id);
    }
}
