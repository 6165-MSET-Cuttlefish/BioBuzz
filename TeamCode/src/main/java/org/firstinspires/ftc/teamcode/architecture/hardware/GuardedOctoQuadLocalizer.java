package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.core.Faults;

public final class GuardedOctoQuadLocalizer implements Localizer {
    public static final String FAULT_KEY = "OctoQuad";
    private static final long RUNNING_TIMEOUT_NANOS = 3_000_000_000L;
    private static final long INVALID_LIMIT_NANOS = 250_000_000L;
    private static final int NO_IMU_READS = 3;

    private final OctoQuad octoQuad;
    private final OctoQuadConfig config;
    private final String name;
    private final DistanceUnit unit;
    private final OctoQuad.LocalizerDataBlock data = new OctoQuad.LocalizerDataBlock();
    private OctoQuad.LocalizerStatus polled;
    private MotionState state = MotionState.zero();
    private Pose pendingPose;
    private boolean configured;
    private boolean running;
    private long resetNanos;
    private long firstInvalidNanos;
    private int noImuReads;

    public GuardedOctoQuadLocalizer(HardwareMap hardwareMap, OctoQuadConfig config) {
        this.config = config;
        name = config.name.get();
        octoQuad = hardwareMap.get(OctoQuad.class, name);
        unit = config.globalDistanceUnit.get();
        configured = octoQuad.getChipId() == OctoQuad.OCTOQUAD_CHIP_ID;
        configure();
        restart();
    }

    public String status() {
        if (running) return data.crcOk ? String.valueOf(data.localizerStatus) : "no valid reply";
        if (polled == null) return "not read yet";
        return polled == OctoQuad.LocalizerStatus.INVALID ? "no reply" : polled.name();
    }

    public OctoQuad.LocalizerDataBlock lastRead() {
        return data;
    }

    public void awaitRunning() {
        long deadline = System.nanoTime() + RUNNING_TIMEOUT_NANOS;
        while (!running) {
            update();
            if (!running && (gaveUp() || System.nanoTime() > deadline)) throw new IllegalStateException(problem());
        }
    }

    @Override
    public void update() {
        if (running) {
            readRunning();
            return;
        }
        polled = octoQuad.getLocalizerStatus();
        if (polled == OctoQuad.LocalizerStatus.INVALID) {
            configured = false;
        } else if (!configured) {
            configured = true;
            Faults.clear(FAULT_KEY);
            configure();
            restart();
            return;
        }
        noImuReads = polled == OctoQuad.LocalizerStatus.FAULT_NO_IMU ? noImuReads + 1 : 0;
        if (polled == OctoQuad.LocalizerStatus.RUNNING) {
            octoQuad.readLocalizerData(data);
            if (data.isDataValid()) {
                running = true;
                Faults.clear(FAULT_KEY);
                if (pendingPose != null) {
                    writePose(pendingPose);
                    state = MotionState.ofVelocity(pendingPose, Velocity.zero());
                    pendingPose = null;
                } else {
                    state = fromData();
                }
                return;
            }
        }
        if (gaveUp()) Faults.raise(FAULT_KEY, problem());
    }

    @Override
    public void setPose(Pose pose) {
        state = state.withPose(pose);
        if (running) writePose(pose);
        else pendingPose = pose;
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void reset() {
        pendingPose = null;
        restart();
    }

    private void readRunning() {
        octoQuad.readLocalizerData(data);
        long now = System.nanoTime();
        if (data.isDataValid()) {
            firstInvalidNanos = 0;
            state = fromData();
            return;
        }
        if (firstInvalidNanos == 0) {
            firstInvalidNanos = now;
        } else if (now - firstInvalidNanos > INVALID_LIMIT_NANOS) {
            throw new IllegalStateException(String.format("OctoQuad \"%s\" (%s) sent no valid localizer data for %.0f ms: %s",
                    name, octoQuad.getConnectionInfo(), (now - firstInvalidNanos) / 1e6, status()));
        }
    }

    private void configure() {
        octoQuad.setSingleEncoderDirection(config.xPodPort.get(), config.xPodDirection.get());
        octoQuad.setSingleEncoderDirection(config.yPodPort.get(), config.yPodDirection.get());

        double mmPerUnit = config.encoderResolutionUnit.get().toMm(1.0);
        float ticksPerMM = (float) (config.ticksPerUnit.get() / mmPerUnit);

        octoQuad.setAllLocalizerParameters(
                config.xPodPort.get(),
                config.yPodPort.get(),
                ticksPerMM,
                ticksPerMM,
                (float) -config.offsetUnits.get().toMm(config.yPodOffset.get()),
                (float) -config.offsetUnits.get().toMm(config.xPodOffset.get()),
                config.headingScalar.get().floatValue(),
                config.localizerVelocityIntervalMS.get()
        );
        octoQuad.setI2cRecoveryMode(config.i2cRecoveryMode.get());
    }

    private void restart() {
        octoQuad.resetLocalizerAndCalibrateIMU();
        resetNanos = System.nanoTime();
        running = false;
        firstInvalidNanos = 0;
        noImuReads = 0;
    }

    private MotionState fromData() {
        return MotionState.ofVelocity(
                new Pose(unit.fromMm(data.posX_mm), unit.fromMm(data.posY_mm), data.heading_rad),
                new Velocity(unit.fromMm(data.velX_mmS), unit.fromMm(data.velY_mmS), data.velHeading_radS));
    }

    private boolean gaveUp() {
        return noImuReads >= NO_IMU_READS || System.nanoTime() - resetNanos > RUNNING_TIMEOUT_NANOS;
    }

    private String problem() {
        String which = "\"" + name + "\" (" + octoQuad.getConnectionInfo() + ")";
        if (noImuReads >= NO_IMU_READS) {
            return "OctoQuad " + which + " reports FAULT_NO_IMU: its localizer needs an OctoQuad MK2";
        }
        if (polled == null || polled == OctoQuad.LocalizerStatus.INVALID) {
            return "OctoQuad " + which + " isn't answering: check its I2C cable and the port in the config";
        }
        if (polled == OctoQuad.LocalizerStatus.RUNNING) {
            return "OctoQuad " + which + " reports RUNNING but its localizer data fails its CRC";
        }
        return "OctoQuad " + which + " is " + polled + ", not RUNNING, " + RUNNING_TIMEOUT_NANOS / 1_000_000_000L
                + " s after its reset";
    }

    private void writePose(Pose pose) {
        octoQuad.setLocalizerPose((int) unit.toMm(pose.x()), (int) unit.toMm(pose.y()), (float) pose.heading());
    }
}
