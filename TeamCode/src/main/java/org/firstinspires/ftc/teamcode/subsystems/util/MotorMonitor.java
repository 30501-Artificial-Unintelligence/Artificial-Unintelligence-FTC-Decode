package org.firstinspires.ftc.teamcode.subsystems.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * MotorMonitor (FAST):
 * - No String.format in loop
 * - Avoids repeated exceptions (detects current support once)
 * - Uses System.nanoTime gating
 * - Can sample current at a slower rate than velocity/power
 * - Emits ONE telemetry item by default to reduce overhead
 */
public class MotorMonitor {

    private static class Entry {
        final String name;
        final DcMotorEx motor;

        // formatting buffer reused (no allocations per update)
        final StringBuilder sb = new StringBuilder(48);

        // current sensing support detected once
        boolean currentSupported = true;

        // current sampling throttle
        long nextCurrentNs = 0;
        double lastCurrentA = Double.NaN;

        Entry(String name, DcMotorEx motor) {
            this.name = name;
            this.motor = motor;
        }
    }

    private final Telemetry telemetry;
    private final List<Entry> motors = new ArrayList<>(16);
    private final List<VoltageSensor> voltageSensors = new ArrayList<>(4);

    // overall update rate
    private long periodNs;
    private long nextUpdateNs = 0;

    // current sampling rate (default slower than main update)
    private long currentPeriodNs;

    private boolean showVelocity = true;
    private boolean showPower = true;
    private boolean showCurrent = true;
    private boolean showBattery = true;

    // output mode
    private boolean singleTelemetryItem = true; // fastest

    // big shared buffer reused each update
    private final StringBuilder out = new StringBuilder(512);

    public MotorMonitor(HardwareMap hardwareMap, Telemetry telemetry, String... motorNames) {
        this(hardwareMap, telemetry, 10.0, motorNames); // default 10 Hz
    }

    public MotorMonitor(HardwareMap hardwareMap, Telemetry telemetry, double hz, String... motorNames) {
        this.telemetry = telemetry;

        setRateHz(hz);
        setCurrentRateHz(5.0); // default current @ 5 Hz

        // Cache voltage sensors
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            voltageSensors.add(vs);
        }

        // Load motors and detect current support ONCE
        for (String name : motorNames) {
            DcMotorEx m;
            try {
                m = hardwareMap.get(DcMotorEx.class, name);
            } catch (Exception e) {
                // skip missing motors instead of crashing
                continue;
            }

            Entry entry = new Entry(name, m);

            // detect if current sensing works (avoid repeated exception cost)
            if (showCurrent) {
                try {
                    m.getCurrent(CurrentUnit.AMPS);
                } catch (Exception ignored) {
                    entry.currentSupported = false;
                }
            }

            motors.add(entry);
        }
    }

    /** Set overall update rate in Hz (e.g., 10). */
    public MotorMonitor setRateHz(double hz) {
        if (hz <= 0) hz = 10.0;
        this.periodNs = (long) (1e9 / hz);
        return this;
    }

    /** Set current sampling rate in Hz (e.g., 5). */
    public MotorMonitor setCurrentRateHz(double hz) {
        if (hz <= 0) hz = 5.0;
        this.currentPeriodNs = (long) (1e9 / hz);
        return this;
    }

    public MotorMonitor setShowVelocity(boolean enabled) { this.showVelocity = enabled; return this; }
    public MotorMonitor setShowPower(boolean enabled) { this.showPower = enabled; return this; }
    public MotorMonitor setShowCurrent(boolean enabled) { this.showCurrent = enabled; return this; }
    public MotorMonitor setShowBattery(boolean enabled) { this.showBattery = enabled; return this; }

    /** If false, it will emit 1 telemetry item per motor (slower). */
    public MotorMonitor setSingleTelemetryItem(boolean enabled) { this.singleTelemetryItem = enabled; return this; }

    /** Call every loop; internally rate-limited. */
    public void update() {
        final long now = System.nanoTime();
        if (now < nextUpdateNs) return;
        nextUpdateNs = now + periodNs;

        if (singleTelemetryItem) {
            out.setLength(0);

            if (showBattery) {
                double batt = getBatteryVoltage();
                out.append("Batt=");
                if (batt > 0) appendFixed2(out, batt);
                else out.append("NA");
                out.append("V\n");
            }

            for (Entry e : motors) {
                e.sb.setLength(0);

                if (showCurrent) {
                    if (!e.currentSupported) {
                        e.sb.append("I=NA");
                    } else {
                        if (now >= e.nextCurrentNs) {
                            e.nextCurrentNs = now + currentPeriodNs;
                            try {
                                e.lastCurrentA = e.motor.getCurrent(CurrentUnit.AMPS);
                            } catch (Exception ex) {
                                e.currentSupported = false;
                                e.lastCurrentA = Double.NaN;
                            }
                        }
                        e.sb.append("I=");
                        if (Double.isNaN(e.lastCurrentA) || Double.isInfinite(e.lastCurrentA)) e.sb.append("NA");
                        else appendFixed2(e.sb, e.lastCurrentA);
                    }
                    e.sb.append("A");
                }

                if (showVelocity) {
                    if (e.sb.length() > 0) e.sb.append("  ");
                    e.sb.append("v=");
                    appendFixed0(e.sb, safeGetVelocity(e.motor));
                }

                if (showPower) {
                    if (e.sb.length() > 0) e.sb.append("  ");
                    e.sb.append("p=");
                    appendFixed2(e.sb, safeGetPower(e.motor));
                }

                out.append(e.name).append(": ").append(e.sb).append('\n');
            }

            telemetry.addData("MM", out.toString());
        } else {
            // slower mode: one telemetry item per motor
            if (showBattery) {
                double batt = getBatteryVoltage();
                telemetry.addData("Batt(V)", (batt > 0) ? fastFixed2(batt) : "NA");
            }

            for (Entry e : motors) {
                e.sb.setLength(0);

                if (showCurrent) {
                    if (!e.currentSupported) {
                        e.sb.append("I=NA");
                    } else {
                        if (now >= e.nextCurrentNs) {
                            e.nextCurrentNs = now + currentPeriodNs;
                            try {
                                e.lastCurrentA = e.motor.getCurrent(CurrentUnit.AMPS);
                            } catch (Exception ex) {
                                e.currentSupported = false;
                                e.lastCurrentA = Double.NaN;
                            }
                        }
                        e.sb.append("I=");
                        if (Double.isNaN(e.lastCurrentA) || Double.isInfinite(e.lastCurrentA)) e.sb.append("NA");
                        else appendFixed2(e.sb, e.lastCurrentA);
                    }
                    e.sb.append("A");
                }

                if (showVelocity) {
                    if (e.sb.length() > 0) e.sb.append("  ");
                    e.sb.append("v=");
                    appendFixed0(e.sb, safeGetVelocity(e.motor));
                }

                if (showPower) {
                    if (e.sb.length() > 0) e.sb.append("  ");
                    e.sb.append("p=");
                    appendFixed2(e.sb, safeGetPower(e.motor));
                }

                telemetry.addData(e.name, e.sb.toString());
            }
        }
    }

    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : voltageSensors) {
            try {
                double v = vs.getVoltage();
                if (v > 0 && v < min) min = v;
            } catch (Exception ignored) {}
        }
        return (min == Double.POSITIVE_INFINITY) ? -1.0 : min;
    }

    private static double safeGetVelocity(DcMotorEx motor) {
        try { return motor.getVelocity(); }
        catch (Exception ignored) { return 0.0; }
    }

    private static double safeGetPower(DcMotorEx motor) {
        try { return motor.getPower(); }
        catch (Exception ignored) { return 0.0; }
    }

    // -------- FAST formatting helpers (no String.format) --------
    private static void appendFixed0(StringBuilder sb, double v) {
        long x = Math.round(v);
        sb.append(x);
    }

    private static void appendFixed2(StringBuilder sb, double v) {
        // handles negatives
        boolean neg = v < 0;
        if (neg) v = -v;

        long scaled = Math.round(v * 100.0);
        long whole = scaled / 100;
        long frac = scaled % 100;

        if (neg) sb.append('-');
        sb.append(whole).append('.');
        if (frac < 10) sb.append('0');
        sb.append(frac);
    }

    private static String fastFixed2(double v) {
        StringBuilder sb = new StringBuilder(12);
        appendFixed2(sb, v);
        return sb.toString();
    }
}
