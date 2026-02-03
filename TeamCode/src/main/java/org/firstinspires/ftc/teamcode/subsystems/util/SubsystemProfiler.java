package org.firstinspires.ftc.teamcode.subsystems.util;

import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Low-overhead per-section profiler.
 * - start/end sections by name
 * - tracks last/ema/max
 * - captures a snapshot when loop spikes over threshold
 */
public class SubsystemProfiler {

    public static boolean ENABLED = true;

    // Report cadence (you can also gate this from TeleOp)
    public static long REPORT_PERIOD_MS = 100;

    // If loop exceeds this, capture a snapshot of all section timings
    public static long SPIKE_THRESHOLD_MS = 18;

    // EMA smoothing for "avg"
    public static double EMA_ALPHA = 0.15;

    private static class Stat {
        long startNs;
        double lastMs;
        double emaMs;
        double maxMs;
    }

    private final LinkedHashMap<String, Stat> stats = new LinkedHashMap<>();

    private long loopStartNs = 0;
    private double loopLastMs = 0;
    private double loopEmaMs = 0;
    private double loopMaxMs = 0;

    private long lastPublishMs = 0;

    // Snapshot captured on spike
    private long lastSpikeLoopMs = 0;
    private final LinkedHashMap<String, Double> spikeSnapshot = new LinkedHashMap<>();

    public void register(String... names) {
        for (String n : names) stats.put(n, new Stat());
    }

    public void loopStart() {
        if (!ENABLED) return;
        loopStartNs = System.nanoTime();
    }

    public void loopEnd() {
        if (!ENABLED) return;
        long nowNs = System.nanoTime();
        loopLastMs = (nowNs - loopStartNs) / 1e6;
        loopEmaMs = (loopEmaMs == 0) ? loopLastMs : (EMA_ALPHA * loopLastMs + (1.0 - EMA_ALPHA) * loopEmaMs);
        if (loopLastMs > loopMaxMs) loopMaxMs = loopLastMs;

        // Spike capture
        if (loopLastMs >= SPIKE_THRESHOLD_MS) {
            lastSpikeLoopMs = (long) Math.round(loopLastMs);
            spikeSnapshot.clear();
            for (Map.Entry<String, Stat> e : stats.entrySet()) {
                spikeSnapshot.put(e.getKey(), e.getValue().lastMs);
            }
        }
    }

    public void start(String name) {
        if (!ENABLED) return;
        Stat s = stats.get(name);
        if (s == null) return;
        s.startNs = System.nanoTime();
    }

    public void end(String name) {
        if (!ENABLED) return;
        Stat s = stats.get(name);
        if (s == null) return;

        double ms = (System.nanoTime() - s.startNs) / 1e6;
        s.lastMs = ms;
        s.emaMs = (s.emaMs == 0) ? ms : (EMA_ALPHA * ms + (1.0 - EMA_ALPHA) * s.emaMs);
        if (ms > s.maxMs) s.maxMs = ms;
    }

    public double getLoopLastMs() { return loopLastMs; }
    public double getLoopEmaMs()  { return loopEmaMs; }
    public double getLoopMaxMs()  { return loopMaxMs; }

    /** Push to Panels + DS at a throttled cadence. */
    public void publish(TelemetryManager telemetryM, Telemetry telemetry, long nowMs) {
        if (!ENABLED) return;
        if (nowMs - lastPublishMs < REPORT_PERIOD_MS) return;
        lastPublishMs = nowMs;

        // Loop summary
        if (telemetryM != null) {
            telemetryM.addData("prof/loop_ms", loopLastMs);
            telemetryM.addData("prof/loop_avg_ms", loopEmaMs);
            telemetryM.addData("prof/loop_max_ms", loopMaxMs);
        }

        // Per section
        for (Map.Entry<String, Stat> e : stats.entrySet()) {
            String n = e.getKey();
            Stat s = e.getValue();
            if (telemetryM != null) {
                telemetryM.addData("prof/" + n + "/ms", s.lastMs);
                telemetryM.addData("prof/" + n + "/avg_ms", s.emaMs);
                telemetryM.addData("prof/" + n + "/max_ms", s.maxMs);
            }
        }

        // Spike snapshot (super useful)
        if (lastSpikeLoopMs > 0 && telemetryM != null) {
            telemetryM.debug("SPIKE loop_ms=" + lastSpikeLoopMs);
            for (Map.Entry<String, Double> e : spikeSnapshot.entrySet()) {
                telemetryM.addData("spike/" + e.getKey(), e.getValue());
            }
        }

        // DS (keep short)
        if (telemetry != null) {
            telemetry.addData("loop(ms)", "%.1f avg %.1f max %.1f", loopLastMs, loopEmaMs, loopMaxMs);
        }
    }
}
