package org.firstinspires.ftc.teamcode.subsystems.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * MotorMonitor: polls motor telemetry at a fixed rate (default 50 Hz) and prints to Telemetry.
 *
 * Usage (in your OpMode init):
 *   MotorMonitor monitor = new MotorMonitor(hardwareMap, telemetry, 50,
 *        "lf","lr","rf","rr","slideL","slideR","intake","arm");
 *
 * In your loop:
 *   monitor.update();  // rate-limited to 50 Hz
 *
 * Notes:
 * - getCurrent(...) requires DcMotorEx and a motor controller that supports current sensing.
 * - If a motor/controller doesn't support current, it will show "NA".
 */
public class MotorMonitor {

    private static class Entry {
        final String name;
        final DcMotorEx motor;

        Entry(String name, DcMotorEx motor) {
            this.name = name;
            this.motor = motor;
        }
    }

    private final Telemetry telemetry;
    private final List<Entry> motors = new ArrayList<>();
    private final List<VoltageSensor> voltageSensors = new ArrayList<>();

    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double nextUpdateTimeSec = 0.0;

    private double hz = 50.0; // default
    private boolean showVelocity = true;
    private boolean showPower = true;
    private boolean showCurrent = true;
    private boolean showBattery = true;

    public MotorMonitor(HardwareMap hardwareMap, Telemetry telemetry, String... motorNames) {
        this(hardwareMap, telemetry, 50.0, motorNames);
    }

    public MotorMonitor(HardwareMap hardwareMap, Telemetry telemetry, double hz, String... motorNames) {
        this.telemetry = telemetry;
        setRateHz(hz);

        // Cache voltage sensors (useful for brownout debugging).
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            voltageSensors.add(vs);
        }

        // Load motors by name as DcMotorEx.
        for (String name : motorNames) {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
            motors.add(new Entry(name, m));
        }
    }

    /** Set update rate in Hz (e.g., 50). */
    public MotorMonitor setRateHz(double hz) {
        if (hz <= 0) hz = 50.0;
        this.hz = hz;
        return this;
    }

    /** Optional toggles to reduce telemetry spam if needed. */
    public MotorMonitor setShowVelocity(boolean enabled) { this.showVelocity = enabled; return this; }
    public MotorMonitor setShowPower(boolean enabled) { this.showPower = enabled; return this; }
    public MotorMonitor setShowCurrent(boolean enabled) { this.showCurrent = enabled; return this; }
    public MotorMonitor setShowBattery(boolean enabled) { this.showBattery = enabled; return this; }

    /** Call every loop; internally rate-limited to requested Hz. */
    public void update() {
        final double now = timer.seconds();
        if (now < nextUpdateTimeSec) return;
        nextUpdateTimeSec = now + (1.0 / hz);

        // Battery voltage (lowest non-zero reading tends to be the real one under load).
        if (showBattery) {
            double batt = getBatteryVoltage();
            if (batt > 0) telemetry.addData("Batt(V)", String.format("%.2f", batt));
            else telemetry.addData("Batt(V)", "NA");
        }

        // Print one line per motor (name: I, vel, pwr)
        for (Entry e : motors) {
            StringBuilder sb = new StringBuilder();

            if (showCurrent) {
                String iStr = readCurrent(e.motor);
                sb.append("I=").append(iStr).append("A");
            }

            if (showVelocity) {
                double vel = safeGetVelocity(e.motor);
                if (sb.length() > 0) sb.append("  ");
                sb.append("v=").append(String.format("%.0f", vel));
            }

            if (showPower) {
                double p = safeGetPower(e.motor);
                if (sb.length() > 0) sb.append("  ");
                sb.append("p=").append(String.format("%.2f", p));
            }

            telemetry.addLine(e.name + ": " + sb);
        }

        telemetry.update();
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

    private String readCurrent(DcMotorEx motor) {
        try {
            // FTC SDK supports CurrentUnit.AMPS
            double amps = motor.getCurrent(CurrentUnit.AMPS);
            if (Double.isNaN(amps) || Double.isInfinite(amps)) return "NA";
            return String.format("%.2f", amps);
        } catch (Exception ignored) {
            // Some motor controllers don't support current sensing
            return "NA";
        }
    }

    private double safeGetVelocity(DcMotorEx motor) {
        try {
            return motor.getVelocity(); // ticks/sec in FTC SDK for DcMotorEx
        } catch (Exception ignored) {
            return 0.0;
        }
    }

    private double safeGetPower(DcMotorEx motor) {
        try {
            return motor.getPower();
        } catch (Exception ignored) {
            return 0.0;
        }
    }
}
