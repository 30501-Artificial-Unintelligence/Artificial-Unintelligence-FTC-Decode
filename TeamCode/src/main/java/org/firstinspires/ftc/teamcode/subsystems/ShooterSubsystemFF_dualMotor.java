package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.subsystems.util.PIDFController;

@Configurable
public class ShooterSubsystemFF_dualMotor {

    // ===== ENCODER / MOTOR CONSTANTS =====
    private static final double TICKS_PER_REV = 28.0;
    private static final double PHYSICAL_MAX_MOTOR_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;

    // ===== GEAR RATIO (motor gear : flywheel gear) =====
    private static final double MOTOR_TO_FLYWHEEL = 4.0 / 6.0;
    private static final double FLYWHEEL_TO_MOTOR = 1.0 / MOTOR_TO_FLYWHEEL;

    private static final double PHYSICAL_MAX_FLYWHEEL_RPM = PHYSICAL_MAX_MOTOR_RPM * MOTOR_TO_FLYWHEEL;

    // ===== HOOD SERVO CONSTANTS =====
    private static final double HOOD_NEAR_POS = 0.3;
    private static final double HOOD_FAR_POS  = 0.3;

    // =========================
    // PANELS TUNABLES (STATIC)
    // =========================
    public static double TUNE_NEAR_RPM = 2300.0;
    public static double TUNE_FAR_RPM  = 3050.0;

    public static double TUNE_HOOD_NEAR_POS = HOOD_NEAR_POS;
    public static double TUNE_HOOD_FAR_POS  = HOOD_FAR_POS;

    public static double TUNE_kP = 0.03;
    public static double TUNE_kI = 0.0;
    public static double TUNE_kD = 0.0;
    public static double TUNE_kV = 0.00042;
    public static double TUNE_kS = 0.05;

    public static boolean TUNE_FORCE_ON = false;
    public static double  TUNE_FORCE_RPM = 3000.0;

    // =========================
    // PERF / CACHING TUNABLES
    // =========================
    public static boolean ENABLE_WRITE_CACHING = true;
    public static double MOTOR_CACHE_TOLERANCE = 0.002; // power tolerance
    public static double SERVO_CACHE_TOLERANCE = 0.001; // position tolerance

    // Velocity smoothing (reduces PID pulsing)
    public static boolean ENABLE_VEL_FILTER = true;
    public static double  VEL_FILTER_ALPHA = 0.25; // 0..1, higher=less smoothing

    // =========================
    // BANG-BANG MODE
    // =========================
    /** Dashboard toggle: if true, bang-bang replaces PIDF. */
    public static boolean TUNE_USE_BANGBANG = false;

    /** Bang-bang: if |error| > this, go full power (fast spin-up). */
    public static double BB_FULL_ON_ERR_RPM = 500.0;

    /** Bang-bang hysteresis band around target (hold near FF). */
    public static double BB_HYST_RPM = 80.0;

    /** Bang-bang extra +/- power step around FF when outside hysteresis but not full-on. */
    public static double BB_STEP_POWER = 0.12;

    /** Clip final power to this max (still Range.clip to [-1..1]). */
    public static double BB_MAX_POWER = 1.0;

    // =========================
    // LIGHT UPDATE CONTROL
    // =========================
    public static boolean LIGHT_ONLY_ON_CHANGE = true;
    public static boolean LIGHT_RATE_LIMIT = true;
    public static long LIGHT_PERIOD_MS = 50;

    // =========================
    // PROFILING
    // =========================
    public static boolean PROFILE = true;

    // ===== HARDWARE =====
    private final MotorEx motor1;
    private final MotorEx motor2;
    private final ServoEx hoodServo;
    private final LightSubsystem light;

    // ===== CONTROLLER =====
    private final PIDFController controller;

    // ===== STATE =====
    private boolean isOn = false;
    private int fieldPos = 0; // 0 near, 1 far

    private double nearRpm = 3050.0;
    private double farRpm  = 3700.0;

    private boolean prevRpmUp   = false;
    private boolean prevRpmDown = false;

    // ===== TUNABLES (INSTANCE COPIES) =====
    private double kP = 0.0, kI = 0.0, kD = 0.0, kV = 0.0, kS = 0.0;

    // Debug / telemetry values
    private double lastTargetMotorTps = 0.0;
    private double lastVelMotorTps = 0.0;
    private double lastPowerCmd = 0.0;
    private double lastFF = 0.0;
    private double lastVelMotor1Tps = 0.0;
    private double lastVelMotor2Tps = 0.0;

    // Filter state
    private double velFilteredTps = 0.0;
    private boolean velFilterInit = false;

    // Command caching
    private double lastPowerSent = 999.0;
    private double lastHoodSent = 999.0;

    // Light caching
    private int lastLightColor = -999;
    private long lastLightMs = 0;

    // Bang-bang runtime toggle (so you can bind to a gamepad toggle)
    private boolean bangbangEnabledRuntime = false;
    public void setBangBangEnabled(boolean enabled) { bangbangEnabledRuntime = enabled; }
    public boolean isBangBangEnabled() { return (TUNE_USE_BANGBANG || bangbangEnabledRuntime); }

    // Profiling breakdown (ms)
    private double profTotalMs = 0;
    private double profSenseMs = 0;
    private double profComputeMs = 0;
    private double profWriteMs = 0;

    public double getUpdateMs() { return profTotalMs; }
    public double getUpdateSenseMs() { return profSenseMs; }
    public double getUpdateComputeMs() { return profComputeMs; }
    public double getUpdateWriteMs() { return profWriteMs; }


    public ShooterSubsystemFF_dualMotor(HardwareMap hardwareMap) {
        motor1 = new MotorEx(hardwareMap, "motor_one", TICKS_PER_REV, PHYSICAL_MAX_MOTOR_RPM);
        motor2 = new MotorEx(hardwareMap, "turretMotor", TICKS_PER_REV, PHYSICAL_MAX_MOTOR_RPM);

        hoodServo = new ServoEx(hardwareMap, "hoodServo");
        light = new LightSubsystem(hardwareMap);

        initMotor(motor1);
        initMotor(motor2);

        motor1.setInverted(true);
        motor2.setInverted(false);

        if (ENABLE_WRITE_CACHING) {
            motor1.setCachingTolerance(MOTOR_CACHE_TOLERANCE);
            motor2.setCachingTolerance(MOTOR_CACHE_TOLERANCE);
            hoodServo.setCachingTolerance(SERVO_CACHE_TOLERANCE);
        }

        syncFromTunables();
        controller = new PIDFController(kP, kI, kD, 0.0);

        fieldPos = 0;
        setHoodIfChanged(TUNE_HOOD_NEAR_POS);
        setMotorsIfChanged(0.0);
        setLightSmart(1, System.currentTimeMillis(), true);
    }

    private void initMotor(MotorEx m) {
        m.stopAndResetEncoder();
        m.setRunMode(Motor.RunMode.RawPower);
        m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    private void syncFromTunables() {
        nearRpm = TUNE_NEAR_RPM;
        farRpm  = TUNE_FAR_RPM;

        kP = TUNE_kP;
        kI = TUNE_kI;
        kD = TUNE_kD;
        kV = TUNE_kV;
        kS = TUNE_kS;
    }

    private void syncToTunables() {
        TUNE_NEAR_RPM = nearRpm;
        TUNE_FAR_RPM  = farRpm;

        TUNE_kP = kP;
        TUNE_kI = kI;
        TUNE_kD = kD;
        TUNE_kV = kV;
        TUNE_kS = kS;
    }

    private static double ticksPerSecToMotorRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }
    private static double motorRpmToTicksPerSec(double rpm) { return rpm * TICKS_PER_REV / 60.0; }
    private static double motorRpmToFlywheelRpm(double rpm) { return rpm * MOTOR_TO_FLYWHEEL; }
    private static double flywheelRpmToMotorRpm(double rpm) { return rpm * FLYWHEEL_TO_MOTOR; }

    public void update(boolean shooterOnCommand,
                       boolean rpmUpButton,
                       boolean rpmDownButton,
                       int fieldPosInput) {

        final long t0 = PROFILE ? System.nanoTime() : 0;
        final long nowMs = System.currentTimeMillis();

        syncFromTunables();

        // === FIELD POS / HOOD (ONLY WRITE IF CHANGED) ===
        int newFieldPos = (fieldPosInput == 1) ? 1 : 0;
        if (newFieldPos != fieldPos) fieldPos = newFieldPos;

        double hoodTarget = (fieldPos == 1) ? TUNE_HOOD_FAR_POS : TUNE_HOOD_NEAR_POS;
        setHoodIfChanged(hoodTarget);

        // === RPM ADJUST (edge) ===
        if (rpmUpButton && !prevRpmUp) {
            if (fieldPos == 1) farRpm  = Math.min(PHYSICAL_MAX_FLYWHEEL_RPM, farRpm + RPM_STEP);
            else               nearRpm = Math.min(PHYSICAL_MAX_FLYWHEEL_RPM, nearRpm + RPM_STEP);
            syncToTunables();
        }
        if (rpmDownButton && !prevRpmDown) {
            if (fieldPos == 1) farRpm  = Math.max(0.0, farRpm - RPM_STEP);
            else               nearRpm = Math.max(0.0, nearRpm - RPM_STEP);
            syncToTunables();
        }
        prevRpmUp = rpmUpButton;
        prevRpmDown = rpmDownButton;

        // === APPLY COMMAND ===
        boolean on = shooterOnCommand || TUNE_FORCE_ON;
        isOn = on;

        double targetFlywheelRpm = TUNE_FORCE_ON ? TUNE_FORCE_RPM : getTargetRpm();

        // EARLY EXIT: if off or target ~0, don’t read velocities, don’t do PID, don’t spam writes
        if (!isOn || targetFlywheelRpm <= 1.0) {
            final long tWrite0 = PROFILE ? System.nanoTime() : 0;
            setMotorsIfChanged(0.0);
            lastPowerCmd = 0.0;
            lastFF = 0.0;
            velFilterInit = false;
            setLightSmart(1, nowMs, false);
            final long tWrite1 = PROFILE ? System.nanoTime() : 0;

            if (PROFILE) {
                long tEnd = System.nanoTime();
                profTotalMs = (tEnd - t0) / 1e6;
                profSenseMs = 0.0;
                profComputeMs = 0.0;
                profWriteMs = (tWrite1 - tWrite0) / 1e6;
            }
            return;
        }

        // Compute target in motor ticks/sec
        double targetMotorRpm = flywheelRpmToMotorRpm(targetFlywheelRpm);
        double targetMotorTps = motorRpmToTicksPerSec(targetMotorRpm);

        lastTargetMotorTps = targetMotorTps;

        // === SENSOR READS (velocity) ===
        final long tSense0 = PROFILE ? System.nanoTime() : 0;
        double v1 = motor1.getVelocity();
        double v2 = motor2.getVelocity();
        final long tSense1 = PROFILE ? System.nanoTime() : 0;

        lastVelMotor1Tps = v1;
        lastVelMotor2Tps = v2;

        double velMotorTps = 0.5 * (v1 + v2);

        // Optional filtering
        double velUsedTps;
        if (ENABLE_VEL_FILTER) {
            if (!velFilterInit) {
                velFilteredTps = velMotorTps;
                velFilterInit = true;
            } else {
                velFilteredTps = (VEL_FILTER_ALPHA * velMotorTps) + ((1.0 - VEL_FILTER_ALPHA) * velFilteredTps);
            }
            velUsedTps = velFilteredTps;
        } else {
            velUsedTps = velMotorTps;
        }
        lastVelMotorTps = velUsedTps;

        // === CONTROL COMPUTE ===
        final long tComp0 = PROFILE ? System.nanoTime() : 0;

        double errorTps = targetMotorTps - velUsedTps;

        double ff = (kV * targetMotorTps) + kS;  // feedforward power
        lastFF = ff;

        double power;
        if (isBangBangEnabled()) {
            // Bang-bang in RPM domain for intuitive tuning
            double curMotorRpm = ticksPerSecToMotorRpm(velUsedTps);
            double curFlywheelRpm = motorRpmToFlywheelRpm(curMotorRpm);
            double errFlywheelRpm = targetFlywheelRpm - curFlywheelRpm;

            if (Math.abs(errFlywheelRpm) > BB_FULL_ON_ERR_RPM) {
                power = 1.0; // full power spin-up
            } else if (Math.abs(errFlywheelRpm) <= BB_HYST_RPM) {
                power = ff; // hold near FF inside hysteresis
            } else {
                // outside hysteresis: nudge around FF
                power = ff + Math.signum(errFlywheelRpm) * BB_STEP_POWER;
            }

            power = Range.clip(power, -BB_MAX_POWER, BB_MAX_POWER);
        } else {
            controller.setPIDF(kP, kI, kD, ff);
            power = controller.calculate(errorTps);
        }

        power = Range.clip(power, -1.0, 1.0);
        lastPowerCmd = power;

        final long tComp1 = PROFILE ? System.nanoTime() : 0;

        // === WRITES (motors + light) ===
        final long tWrite0 = PROFILE ? System.nanoTime() : 0;
        setMotorsIfChanged(power);

        // Light logic based on flywheel rpm error
        double curMotorRpm = ticksPerSecToMotorRpm(velUsedTps);
        double curFlywheelRpm = motorRpmToFlywheelRpm(curMotorRpm);
        double errFlywheelRpm = targetFlywheelRpm - curFlywheelRpm;
        setLightSmart(Math.abs(errFlywheelRpm) < 150 ? 3 : 2, nowMs, false);

        final long tWrite1 = PROFILE ? System.nanoTime() : 0;

        if (PROFILE) {
            long tEnd = System.nanoTime();
            profTotalMs = (tEnd - t0) / 1e6;
            profSenseMs = (tSense1 - tSense0) / 1e6;
            profComputeMs = (tComp1 - tComp0) / 1e6;
            profWriteMs = (tWrite1 - tWrite0) / 1e6;
        }
    }

    // ===== SMART WRITES =====

    private void setMotorsIfChanged(double pwr) {
        // If SolversLib caching tolerance is enabled, this helps already,
        // but we also avoid calling set() if unchanged to reduce traffic further.
        if (Math.abs(pwr - lastPowerSent) < MOTOR_CACHE_TOLERANCE) return;
        motor1.set(pwr);
        motor2.set(pwr);
        lastPowerSent = pwr;
    }

    private void setHoodIfChanged(double pos) {
        if (Math.abs(pos - lastHoodSent) < SERVO_CACHE_TOLERANCE) return;
        hoodServo.set(pos);
        lastHoodSent = pos;
    }

    private void setLightSmart(int color, long nowMs, boolean force) {
        if (!force) {
            if (LIGHT_RATE_LIMIT && (nowMs - lastLightMs) < LIGHT_PERIOD_MS) return;
            if (LIGHT_ONLY_ON_CHANGE && color == lastLightColor) return;
        }
        light.setColor(color);
        lastLightColor = color;
        lastLightMs = nowMs;
    }

    // ===== GETTERS =====
    public boolean isOn() { return isOn; }
    public int getFieldPos() { return fieldPos; }
    public double getTargetRpm() { return (fieldPos == 1) ? farRpm : nearRpm; }

    public double getTargetMotorTps() { return lastTargetMotorTps; }
    public double getVelocityMotorTps() { return lastVelMotorTps; }
    public double getMotor1VelocityTps() { return lastVelMotor1Tps; }
    public double getMotor2VelocityTps() { return lastVelMotor2Tps; }

    public double getVelocityRpm() {
        double motorRpm = ticksPerSecToMotorRpm(lastVelMotorTps);
        return motorRpmToFlywheelRpm(motorRpm);
    }

    public double getPowerCmd() { return lastPowerCmd; }
    public double getFF() { return lastFF; }

    public void stop() {
        isOn = false;
        setMotorsIfChanged(0.0);
        setLightSmart(1, System.currentTimeMillis(), true);
        velFilterInit = false;
    }

    public double getActiveTargetRpm() {
        return TUNE_FORCE_ON ? TUNE_FORCE_RPM : getTargetRpm();
    }

    public double getCurrentRpmEstimate() {
        double motorRpm = ticksPerSecToMotorRpm(lastVelMotorTps);
        return motorRpmToFlywheelRpm(motorRpm);
    }

    public double getErrorRpm() {
        return getActiveTargetRpm() - getCurrentRpmEstimate();
    }

    public double getKv() { return kV; }
    public double getKs() { return kS; }
}
