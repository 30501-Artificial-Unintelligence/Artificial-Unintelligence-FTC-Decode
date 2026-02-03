package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * PASSIVE Spindexer (frame-mounted sensors, trust only when aligned at intake).
 *
 * Performance changes:
 * - Cache motor ticks/angle ONCE per update()
 * - Reuse storage arrays (no per-loop allocations)
 * - Optional sensor gating + I2C disable
 * - Optional spindexer profiling (ms)
 */
@com.bylazar.configurables.annotations.Configurable
public class SpindexerSubsystem_Passive_State_new_Incremental {

    // =========================
    // ===== CONFIG / TUNABLES ==
    // =========================

    public static boolean REVERSE_MOTOR = true;

    // PIDF
    public static double kP = 0.008;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.12;

    public static double I_CLAMP = 200.0;

    // ===== LOOP-TIME / SENSOR CONTROL =====

    /** Master switch: if false, slot0 Rev sensors are NEVER read (safe if sensors missing). */
    public static boolean ENABLE_SLOT0_I2C = false;

    /** If true, slot0 I2C is skipped whenever i2cAllowed is false (TeleOp sets this). */
    public static boolean SKIP_SLOT0_I2C_WHEN_SHOOTER_ON = true;

    /** If true, skip ALL sensing (digital + I2C) when i2cAllowed is false. */
    public static boolean SKIP_ALL_SENSORS_WHEN_SHOOTER_ON = true;

    /** Optional throttling (0 = no throttle; read as fast as loop runs). */
    public static long SLOT0_I2C_PERIOD_MS = 0;

    /** Internal spindexer debug telemetry (NOT recommended in comp). */
    public static boolean ENABLE_DEBUG_TELEM = false;

    /** Rate-limit debug telemetry calls inside spindexer. */
    public static long DEBUG_TELEM_PERIOD_MS = 100;

    /** Enable internal timing breakdown. */
    public static boolean PROFILE = true;

    // ===== Panels slot override =====
    public static boolean USE_PANELS_SLOT_OVERRIDE = false;
    /** 0=EMPTY, 1=GREEN, 2=PURPLE */
    public static int OVERRIDE_SLOT0 = 0;
    public static int OVERRIDE_SLOT1 = 0;
    public static int OVERRIDE_SLOT2 = 0;

    public static boolean OVERRIDE_ONLY_WHEN_ALIGNED = true;
    public static boolean OVERRIDE_FORCE_SLOT0_REREAD = true;

    // Motor encoder
    private static final double TICKS_PER_REV = 4000.0;
    private static final double DEG_PER_TICK = 360.0 / TICKS_PER_REV;

    // Geometry
    private static final int SLOT_COUNT = 3;
    private static final double DEGREES_PER_SLOT = 120.0;

    // World angles
    public static double INTAKE_ANGLE_DEG   = 0.0;
    public static double PRESHOOT_ANGLE_DEG = 200.0;
    public static double GO_PAST_ANGLE_DEG  = 40.0;

    public static double ALIGN_TOL_DEG = 10.0;

    // Slot0 distance threshold
    public static double SLOT0_DIST_THRESH_CM = 3.0;

    public static int PRESENT_FRAMES_REQUIRED = 2;
    public static int COLOR_FRAMES_REQUIRED   = 2;

    public static double MOVE_MAX_POWER = 0.55;
    public static double MOVE_DONE_TOL_DEG = 2.0;

    public static double OVERSHOOT_CORRECT_MAX_DEG = 60.0;

    public static double HOLD_MAX_POWER = 0.30;

    // Shooting
    public static double SHOOT_DEG = 1080.0;
    public static double SHOOT_POWER_HIGH = 1.0;
    public static double SHOOT_POWER_LOW  = 0.9;
    public static long   SHOOT_HIGH_MS    = 300;
    public static long   SHOOT_TIMEOUT_MS = 2500;

    public static int patternTagOverride = -1;

    // =========================
    // ===== DEBUG (existing) ===
    // =========================
    private double dbgLastRequestedPower = 0.0;
    private double dbgLastAppliedPower   = 0.0;
    private double dbgLastErrShortestDeg = 0.0;
    private double dbgLastErrUsedDeg     = 0.0;

    public double dbgLastRequestedPower() { return dbgLastRequestedPower; }
    public double dbgLastAppliedPower()   { return dbgLastAppliedPower; }
    public double dbgLastErrShortestDeg() { return dbgLastErrShortestDeg; }
    public double dbgLastErrUsedDeg()     { return dbgLastErrUsedDeg; }

    // =========================
    // ===== SPINDEXER PROFILING
    // =========================
    private double profTotalMs = 0;
    private double profHwMs = 0;
    private double profSenseMs = 0;
    private double profMotionMs = 0;
    private double profPersistMs = 0;

    public double getUpdateMs() { return profTotalMs; }
    public double getUpdateMsHw() { return profHwMs; }
    public double getUpdateMsSense() { return profSenseMs; }
    public double getUpdateMsMotion() { return profMotionMs; }
    public double getUpdateMsPersist() { return profPersistMs; }

    private long lastDebugTelemMs = 0;

    // =========================
    // ===== HARDWARE NAMES =====
    // =========================
    public static String SLOT1_PRESENT_NAME = "slot1Present";
    public static String SLOT1_GREEN_NAME   = "slot1Green";
    public static String SLOT2_PRESENT_NAME = "slot2Present";
    public static String SLOT2_GREEN_NAME   = "slot2Green";

    // =========================
    // ===== SLOT MODEL =========
    // =========================
    public enum Ball { EMPTY, GREEN, PURPLE, UNKNOWN }
    private enum RawColor { RED, GREEN, PURPLE }

    private static final int[][] EJECT_ORDER = new int[][]{
            {0, 2, 1},
            {1, 0, 2},
            {2, 1, 0}
    };

    // =========================
    // ===== STATE MACHINE ======
    // =========================
    private enum State {
        IDLE,
        POSITION_TO_PARK_CW,
        POSITION_TO_PRESHOOT_CCW,
        READY,
        SHOOTING,
        RETURNING_CW
    }

    private State state = State.IDLE;
    private State lastState = null;

    // =========================
    // ===== HARDWARE ===========
    // =========================
    private final DcMotorEx motor;

    // These may be null if missing
    private final RevColorSensorV3 slot0Color1;
    private final RevColorSensorV3 slot0Color2;

    private final DigitalChannel slot1Present;
    private final DigitalChannel slot1Green;
    private final DigitalChannel slot2Present;
    private final DigitalChannel slot2Green;

    // =========================
    // ===== ANGLE / ZERO =======
    // =========================
    private int zeroTicks = 0;
    private double targetAngleDeg = 0.0;

    static class SpindexerOpModeStorage {
        static Integer zeroTicks = null;
        static Double  targetAngleDeg = null;
        static Integer gameTag = null;
        static int[]   slotsEnc = null;
        static Boolean reverseMotor = null;
    }

    // =========================
    // ===== SLOT STATE =========
    // =========================
    private final Ball[] slots = new Ball[SLOT_COUNT];

    private int slot0PresentFrames = 0;
    private int slot0ColorFrames = 0;
    private Ball slot0LastColorFrame = Ball.EMPTY;

    private final int[] presentFrames = new int[SLOT_COUNT];
    private final int[] colorFrames = new int[SLOT_COUNT];
    private final Ball[] lastColorFrame = new Ball[SLOT_COUNT];

    private boolean slot0NeedsReread = false;

    private int gameTag = 0;

    private int selectedStartSlot = 0;
    private double parkWheelAngleDeg = 0.0;
    private double preshootWheelAngleDeg = 0.0;

    private boolean shootRequested = false;

    private boolean ejecting = false;
    private long shootStartMs = 0;
    private int shootLastTicks = 0;
    private double shootMovedDeg = 0.0;

    // =========================
    // ===== PID STATE ==========
    // =========================
    private double pidIntegral = 0.0;
    private double pidLastErrDeg = 0.0;
    private long pidLastNs = System.nanoTime();

    private void resetDrivePID() {
        pidIntegral = 0.0;
        pidLastErrDeg = 0.0;
        pidLastNs = System.nanoTime();
    }

    private enum ErrorMode { SHORTEST, PREFER_CW, PREFER_CCW }

    // TeleOp can set this (ex: disable sensors while shooter is running)
    private boolean i2cAllowed = true;
    public void setI2cAllowed(boolean allowed) { this.i2cAllowed = allowed; }

    // Slot0 cached frame + optional throttling time
    private long slot0LastI2cMs = 0;

    private static class Slot0Frame {
        boolean present;
        Ball color;
    }
    private final Slot0Frame slot0Frame = new Slot0Frame(); // REUSED, no allocations

    // =========================
    // ===== ENCODER CACHE ======
    // =========================
    private boolean loopCacheValid = false;
    private int loopTicks = 0;
    private double loopAngleDeg = 0.0;

    private void updateLoopCache() {
        loopTicks = motor.getCurrentPosition();   // ONE hub read per update()
        loopAngleDeg = ticksToAngle(loopTicks);   // pure math
        loopCacheValid = true;
    }

    private int getTicksCached() {
        return loopCacheValid ? loopTicks : motor.getCurrentPosition();
    }

    public double getCurrentAngleDeg() {
        return loopCacheValid ? loopAngleDeg : ticksToAngle(motor.getCurrentPosition());
    }

    // =========================
    // ===== OVERRIDE tracking ==
    // =========================
    private int lastOv0 = -999, lastOv1 = -999, lastOv2 = -999;

    private Ball decodeOverride(int v) {
        if (v == 1) return Ball.GREEN;
        if (v == 2) return Ball.PURPLE;
        return Ball.EMPTY;
    }

    // =========================
    // ===== CONSTRUCTOR ========
    // =========================
    public SpindexerSubsystem_Passive_State_new_Incremental(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slot0Color1 = safeGetRev(hardwareMap, "spindexerIntakeColorSensor1");
        slot0Color2 = safeGetRev(hardwareMap, "spindexerIntakeColorSensor2");

        slot1Present = hardwareMap.get(DigitalChannel.class, SLOT1_PRESENT_NAME);
        slot1Green   = hardwareMap.get(DigitalChannel.class, SLOT1_GREEN_NAME);
        slot2Present = hardwareMap.get(DigitalChannel.class, SLOT2_PRESENT_NAME);
        slot2Green   = hardwareMap.get(DigitalChannel.class, SLOT2_GREEN_NAME);

        slot1Present.setMode(DigitalChannel.Mode.INPUT);
        slot1Green.setMode(DigitalChannel.Mode.INPUT);
        slot2Present.setMode(DigitalChannel.Mode.INPUT);
        slot2Green.setMode(DigitalChannel.Mode.INPUT);

        for (int i = 0; i < SLOT_COUNT; i++) {
            slots[i] = Ball.EMPTY;
            presentFrames[i] = 0;
            colorFrames[i] = 0;
            lastColorFrame[i] = Ball.EMPTY;
        }

        restoreFromStorageOrDefaultToIntake();

        // Hold where we are on boot
        targetAngleDeg = getCurrentAngleDeg();
        state = State.IDLE;
        ejecting = false;
        lastState = state;
        resetDrivePID();
    }

    private void restoreFromStorageOrDefaultToIntake() {
        if (SpindexerOpModeStorage.zeroTicks != null) {
            zeroTicks = SpindexerOpModeStorage.zeroTicks;
            if (SpindexerOpModeStorage.targetAngleDeg != null) targetAngleDeg = SpindexerOpModeStorage.targetAngleDeg;
            if (SpindexerOpModeStorage.gameTag != null) gameTag = SpindexerOpModeStorage.gameTag;

            if (SpindexerOpModeStorage.slotsEnc != null && SpindexerOpModeStorage.slotsEnc.length == SLOT_COUNT) {
                for (int i = 0; i < SLOT_COUNT; i++) slots[i] = decodeSlot(SpindexerOpModeStorage.slotsEnc[i]);
            }
        } else {
            homeSlot0AtIntakeHere();
        }

        if (SpindexerOpModeStorage.reverseMotor != null && SpindexerOpModeStorage.reverseMotor != REVERSE_MOTOR) {
            homeSlot0AtIntakeHere();
        }
    }

    // =========================
    // ===== STORAGE (NO ALLOC) ==
    // =========================
    private void persistToStorage() {
        SpindexerOpModeStorage.zeroTicks = zeroTicks;
        SpindexerOpModeStorage.targetAngleDeg = targetAngleDeg;
        SpindexerOpModeStorage.gameTag = gameTag;
        SpindexerOpModeStorage.reverseMotor = REVERSE_MOTOR;

        // REUSE array, do not allocate each loop
        if (SpindexerOpModeStorage.slotsEnc == null || SpindexerOpModeStorage.slotsEnc.length != SLOT_COUNT) {
            SpindexerOpModeStorage.slotsEnc = new int[SLOT_COUNT];
        }
        int[] enc = SpindexerOpModeStorage.slotsEnc;
        for (int i = 0; i < SLOT_COUNT; i++) enc[i] = encodeSlot(slots[i]);
    }

    private int encodeSlot(Ball b) {
        if (b == Ball.GREEN) return 1;
        if (b == Ball.PURPLE) return 2;
        if (b == Ball.UNKNOWN) return 3;
        return 0;
    }

    private Ball decodeSlot(int v) {
        if (v == 1) return Ball.GREEN;
        if (v == 2) return Ball.PURPLE;
        if (v == 3) return Ball.UNKNOWN;
        return Ball.EMPTY;
    }

    // =========================
    // ===== PUBLIC API =========
    // =========================
    public Ball[] getSlots() { return slots; }

    public boolean hasAnyBall() {
        for (Ball b : slots) if (b != Ball.EMPTY) return true;
        return false;
    }

    public boolean isFull() {
        for (Ball b : slots) if (b == Ball.EMPTY) return false;
        return true;
    }

    public boolean isEjecting() { return ejecting; }

    public int getGameTag() { return gameTag; }
    public void setGameTag(int tag) { this.gameTag = tag; }

    public String getGamePattern() {
        Ball[] p = getPatternForTag(gameTag);
        if (p == null) return "Fast (no pattern, tag=" + gameTag + ")";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < p.length; i++) {
            sb.append(p[i] == Ball.PURPLE ? "P" : (p[i] == Ball.GREEN ? "G" : "?"));
        }
        return sb.toString();
    }

    public void homeToIntake() {
        shootRequested = false;
        ejecting = false;
        state = State.RETURNING_CW;
        targetAngleDeg = normalizeAngle(INTAKE_ANGLE_DEG);
    }

    public void homeSlot0AtIntakeHere() {
        zeroTicks = motor.getCurrentPosition();
        targetAngleDeg = normalizeAngle(INTAKE_ANGLE_DEG);
        state = State.IDLE;

        clearAllSensingCounters();
        slot0NeedsReread = true;

        resetDrivePID();
        persistToStorage();
    }

    public void homeSlot0AtLoadHere() {
        targetAngleDeg = normalizeAngle(PRESHOOT_ANGLE_DEG);
        state = State.IDLE;
        resetDrivePID();
        persistToStorage();
    }

    public int getEncoder() { return motor.getCurrentPosition(); }
    public double getTargetAngleDeg() { return targetAngleDeg; }
    public int getTarget() { return angleToTicks(targetAngleDeg); }

    public double getAngleErrorToTargetDeg() {
        return smallestAngleDiff(targetAngleDeg, getCurrentAngleDeg());
    }

    public boolean isAutoRotating() {
        return state == State.POSITION_TO_PARK_CW
                || state == State.POSITION_TO_PRESHOOT_CCW
                || state == State.RETURNING_CW;
    }

    public int getIntakeIndex() { return 0; }

    public void presetSlots(Ball s0, Ball s1, Ball s2) {
        slots[0] = (s0 == null) ? Ball.EMPTY : s0;
        slots[1] = (s1 == null) ? Ball.EMPTY : s1;
        slots[2] = (s2 == null) ? Ball.EMPTY : s2;
        persistToStorage();
    }

    // =========================
    // ===== MAIN UPDATE ========
    // =========================
    public boolean update(Telemetry telemetry,
                          LoaderSubsystem loader,
                          boolean yEdge,
                          int patternTagOverrideFromCode) {

        final long t0 = PROFILE ? System.nanoTime() : 0;

        // Cache encoder/angle ONCE per loop (biggest win)
        final long tHw0 = PROFILE ? System.nanoTime() : 0;
        updateLoopCache();
        final long tHw1 = PROFILE ? System.nanoTime() : 0;

        long nowMs = System.currentTimeMillis();

        if (yEdge) shootRequested = true;

        // Panels override (no sensor reads)
        if (USE_PANELS_SLOT_OVERRIDE) {
            double errToIntake = smallestAngleDiff(normalizeAngle(INTAKE_ANGLE_DEG), getCurrentAngleDeg());
            boolean allow = (!OVERRIDE_ONLY_WHEN_ALIGNED) || (Math.abs(errToIntake) <= ALIGN_TOL_DEG);

            if (allow) {
                int o0 = OVERRIDE_SLOT0;
                int o1 = OVERRIDE_SLOT1;
                int o2 = OVERRIDE_SLOT2;

                boolean s1New = (lastOv1 == 0) && (o1 != 0);
                boolean s2New = (lastOv2 == 0) && (o2 != 0);

                slots[0] = decodeOverride(o0);
                slots[1] = decodeOverride(o1);
                slots[2] = decodeOverride(o2);

                if (OVERRIDE_FORCE_SLOT0_REREAD && (s1New || s2New)) slot0NeedsReread = true;

                lastOv0 = o0; lastOv1 = o1; lastOv2 = o2;
            }
        }

        // Apply tag overrides
        int effectiveTag = -1;
        boolean panelsValid = (patternTagOverride == 0 || patternTagOverride == 21 || patternTagOverride == 22 || patternTagOverride == 23);
        boolean codeValid   = (patternTagOverrideFromCode == 0 || patternTagOverrideFromCode == 21 || patternTagOverrideFromCode == 22 || patternTagOverrideFromCode == 23);
        if (panelsValid) effectiveTag = patternTagOverride;
        else if (codeValid) effectiveTag = patternTagOverrideFromCode;
        if (effectiveTag != -1) gameTag = effectiveTag;

        // Sensing only if aligned and not shooting, and allowed
        final long tSense0 = PROFILE ? System.nanoTime() : 0;
        boolean aligned = isAlignedAtIntake();
        boolean sensorsAllowed = !(SKIP_ALL_SENSORS_WHEN_SHOOTER_ON && !i2cAllowed);

        if (aligned && state != State.SHOOTING && sensorsAllowed) {
            updateSensorsAligned(nowMs);
        }
        final long tSense1 = PROFILE ? System.nanoTime() : 0;

        // Planning rules
        if (state == State.IDLE && isFull()) {
            planToPreshootFromCurrent();
            state = State.POSITION_TO_PARK_CW;
        }
        if (state == State.IDLE && shootRequested && hasAnyBall() && !isFull()) {
            planToPreshootFromCurrent();
            state = State.POSITION_TO_PARK_CW;
        }

        if (lastState != state) {
            resetDrivePID();
            lastState = state;
        }

        // Motion / state machine
        final long tMotion0 = PROFILE ? System.nanoTime() : 0;
        switch (state) {
            case IDLE: {
                ejecting = false;
                holdToTargetShortestPath();
                break;
            }
            case POSITION_TO_PARK_CW: {
                ejecting = false;
                if (moveTowardTargetCW(parkWheelAngleDeg)) state = State.POSITION_TO_PRESHOOT_CCW;
                break;
            }
            case POSITION_TO_PRESHOOT_CCW: {
                ejecting = false;
                if (moveTowardTargetCCW(preshootWheelAngleDeg)) state = State.READY;
                break;
            }
            case READY: {
                ejecting = false;
                holdToTargetShortestPath();
                if (shootRequested) {
                    beginShooting(nowMs);
                    state = State.SHOOTING;
                }
                break;
            }
            case SHOOTING: {
                ejecting = true;
                if (updateShooting(nowMs)) {
                    slots[0] = Ball.EMPTY;
                    slots[1] = Ball.EMPTY;
                    slots[2] = Ball.EMPTY;

                    state = State.RETURNING_CW;
                    targetAngleDeg = normalizeAngle(INTAKE_ANGLE_DEG);

                    clearAllSensingCounters();
                    slot0NeedsReread = true;
                }
                break;
            }
            case RETURNING_CW: {
                ejecting = false;
                if (moveTowardTargetCW(normalizeAngle(INTAKE_ANGLE_DEG))) state = State.IDLE;
                break;
            }
        }
        final long tMotion1 = PROFILE ? System.nanoTime() : 0;

        // Persist (no alloc)
        final long tPersist0 = PROFILE ? System.nanoTime() : 0;
        persistToStorage();
        final long tPersist1 = PROFILE ? System.nanoTime() : 0;

        // OPTIONAL internal telemetry (rate-limited)
        if (telemetry != null && ENABLE_DEBUG_TELEM) {
            if (nowMs - lastDebugTelemMs >= DEBUG_TELEM_PERIOD_MS) {
                lastDebugTelemMs = nowMs;
                telemetry.addData("Spd/state", state);
                telemetry.addData("Spd/aligned", aligned);
                telemetry.addData("Spd/angle", "%.1f", getCurrentAngleDeg());
                telemetry.addData("Spd/target", "%.1f", targetAngleDeg);
                telemetry.addData("Spd/tag", gameTag);
                telemetry.addData("Spd/slots", "%s %s %s", slots[0], slots[1], slots[2]);
                telemetry.addData("Spd/shootReq", shootRequested);
                telemetry.addData("Spd/pwrSet", "%.2f", dbgLastAppliedPower);
                telemetry.addData("Spd/eShort", "%.1f", dbgLastErrShortestDeg);
                telemetry.addData("Spd/eUsed", "%.1f", dbgLastErrUsedDeg);

                if (PROFILE) {
                    telemetry.addData("Spd/ms", "%.2f", profTotalMs);
                    telemetry.addData("Spd/ms_hw", "%.2f", profHwMs);
                    telemetry.addData("Spd/ms_sense", "%.2f", profSenseMs);
                    telemetry.addData("Spd/ms_motion", "%.2f", profMotionMs);
                    telemetry.addData("Spd/ms_persist", "%.2f", profPersistMs);
                }
            }
        }

        // Save profiling
        if (PROFILE) {
            long tEnd = System.nanoTime();
            profTotalMs   = (tEnd - t0) / 1e6;
            profHwMs      = (tHw1 - tHw0) / 1e6;
            profSenseMs   = (tSense1 - tSense0) / 1e6;
            profMotionMs  = (tMotion1 - tMotion0) / 1e6;
            profPersistMs = (tPersist1 - tPersist0) / 1e6;
        }

        // Clear cache validity after update so outside calls still work
        loopCacheValid = false;

        return isFull();
    }

    // =========================
    // ===== PLANNING ===========
    // =========================
    private void planToPreshootFromCurrent() {
        selectedStartSlot = chooseStartSlotForCurrentTag();
        double parkWorld = normalizeAngle(PRESHOOT_ANGLE_DEG + GO_PAST_ANGLE_DEG);

        parkWheelAngleDeg     = wheelAngleForPocketAtWorldAngle(selectedStartSlot, parkWorld);
        preshootWheelAngleDeg = wheelAngleForPocketAtWorldAngle(selectedStartSlot, PRESHOOT_ANGLE_DEG);
    }

    private int chooseStartSlotForCurrentTag() {
        int[] candidates = new int[]{0, 1, 2};
        Ball[] desired = getPatternForTag(gameTag);

        double cur = getCurrentAngleDeg();
        double parkWorld = normalizeAngle(PRESHOOT_ANGLE_DEG + GO_PAST_ANGLE_DEG);

        int bestSlot = 0;
        int bestScore = -1;
        double bestCw = Double.MAX_VALUE;

        for (int s : candidates) {
            if (slots[s] == Ball.EMPTY) continue;

            int score = 0;
            if (desired != null) {
                int[] order = EJECT_ORDER[s];
                score += matchScore(desired, order);
            }

            double parkWheel = wheelAngleForPocketAtWorldAngle(s, parkWorld);
            double cw = cwDeltaDeg(cur, parkWheel);

            if (score > bestScore || (score == bestScore && cw < bestCw)) {
                bestScore = score;
                bestCw = cw;
                bestSlot = s;
            }
        }

        if (bestScore < 0) {
            bestSlot = 0;
            bestCw = Double.MAX_VALUE;
            for (int s : candidates) {
                if (slots[s] == Ball.EMPTY) continue;
                double parkWheel = wheelAngleForPocketAtWorldAngle(s, parkWorld);
                double cw = cwDeltaDeg(cur, parkWheel);
                if (cw < bestCw) { bestCw = cw; bestSlot = s; }
            }
        }

        return bestSlot;
    }

    private int matchScore(Ball[] desired, int[] order) {
        int score = 0;
        Ball a0 = slots[order[0]];
        Ball a1 = slots[order[1]];
        Ball a2 = slots[order[2]];

        if (a0 != Ball.EMPTY && desired[0] == a0) score += 4;
        if (a1 != Ball.EMPTY && desired[1] == a1) score += 2;
        if (a2 != Ball.EMPTY && desired[2] == a2) score += 1;

        return score;
    }

    private Ball[] getPatternForTag(int tag) {
        Ball[] seq = new Ball[SLOT_COUNT];
        switch (tag) {
            case 23: seq[0] = Ball.PURPLE; seq[1] = Ball.PURPLE; seq[2] = Ball.GREEN;  return seq;
            case 22: seq[0] = Ball.PURPLE; seq[1] = Ball.GREEN;  seq[2] = Ball.PURPLE; return seq;
            case 21: seq[0] = Ball.GREEN;  seq[1] = Ball.PURPLE; seq[2] = Ball.PURPLE; return seq;
            default: return null;
        }
    }

    private double wheelAngleForPocketAtWorldAngle(int slotIndex, double worldAngleDeg) {
        return normalizeAngle(worldAngleDeg - slotIndex * DEGREES_PER_SLOT);
    }

    // =========================
    // ===== SENSING ============
    // =========================
    private void clearAllSensingCounters() {
        slot0PresentFrames = 0;
        slot0ColorFrames = 0;
        slot0LastColorFrame = Ball.EMPTY;

        for (int i = 0; i < SLOT_COUNT; i++) {
            presentFrames[i] = 0;
            colorFrames[i] = 0;
            lastColorFrame[i] = Ball.EMPTY;
        }
    }

    private void updateSensorsAligned(long nowMs) {
        // Slot1/2 digital reads ONLY if slot empty and allowed
        if (slots[1] == Ball.EMPTY) {
            boolean p1 = slot1Present.getState();
            boolean g1 = slot1Green.getState();
            updateBrushlandSlot(1, p1, g1);
        }
        if (slots[2] == Ball.EMPTY) {
            boolean p2 = slot2Present.getState();
            boolean g2 = slot2Green.getState();
            updateBrushlandSlot(2, p2, g2);
        }

        // Slot0 reread
        if (slot0NeedsReread) {
            slots[0] = Ball.EMPTY;
            slot0PresentFrames = 0;
            slot0ColorFrames = 0;
            slot0LastColorFrame = Ball.EMPTY;
            slot0NeedsReread = false;
        }

        if (slots[0] == Ball.EMPTY) {
            Slot0Frame f = readSlot0Frame(nowMs);
            boolean present = f.present;
            Ball frameColor = f.color;

            if (!present) {
                slot0PresentFrames = 0;
                slot0ColorFrames = 0;
                slot0LastColorFrame = Ball.EMPTY;
            } else {
                slot0PresentFrames++;

                if (frameColor == Ball.GREEN || frameColor == Ball.PURPLE) {
                    if (frameColor == slot0LastColorFrame) slot0ColorFrames++;
                    else { slot0LastColorFrame = frameColor; slot0ColorFrames = 1; }
                } else {
                    slot0LastColorFrame = Ball.EMPTY;
                    slot0ColorFrames = 0;
                }

                if (slot0PresentFrames >= PRESENT_FRAMES_REQUIRED && slot0ColorFrames >= COLOR_FRAMES_REQUIRED) {
                    slots[0] = slot0LastColorFrame;
                    slot0PresentFrames = 0;
                    slot0ColorFrames = 0;
                    slot0LastColorFrame = Ball.EMPTY;
                }
            }
        }
    }

    private void updateBrushlandSlot(int slotIndex, boolean presentRaw, boolean greenRaw) {
        if (slotIndex != 1 && slotIndex != 2) return;
        if (slots[slotIndex] != Ball.EMPTY) return;

        boolean present = presentRaw;
        Ball frameColor = Ball.EMPTY;

        if (present) frameColor = greenRaw ? Ball.GREEN : Ball.PURPLE;

        if (!present) {
            presentFrames[slotIndex] = 0;
            colorFrames[slotIndex] = 0;
            lastColorFrame[slotIndex] = Ball.EMPTY;
            return;
        }

        presentFrames[slotIndex]++;

        if (frameColor == Ball.GREEN || frameColor == Ball.PURPLE) {
            if (frameColor == lastColorFrame[slotIndex]) colorFrames[slotIndex]++;
            else { lastColorFrame[slotIndex] = frameColor; colorFrames[slotIndex] = 1; }
        } else {
            lastColorFrame[slotIndex] = Ball.EMPTY;
            colorFrames[slotIndex] = 0;
        }

        if (presentFrames[slotIndex] >= PRESENT_FRAMES_REQUIRED && colorFrames[slotIndex] >= COLOR_FRAMES_REQUIRED) {
            slots[slotIndex] = lastColorFrame[slotIndex];

            // IMPORTANT: reread slot0 when slot1/2 becomes filled
            slot0NeedsReread = true;

            presentFrames[slotIndex] = 0;
            colorFrames[slotIndex] = 0;
            lastColorFrame[slotIndex] = Ball.EMPTY;
        }
    }

    private Slot0Frame readSlot0Frame(long nowMs) {
        // Default
        slot0Frame.present = false;
        slot0Frame.color = Ball.EMPTY;

        // Master switch: never touch I2C
        if (!ENABLE_SLOT0_I2C) return slot0Frame;

        // Shooter gate
        if (SKIP_SLOT0_I2C_WHEN_SHOOTER_ON && !i2cAllowed) return slot0Frame;

        // Missing sensors
        if (slot0Color1 == null || slot0Color2 == null) return slot0Frame;

        // Optional throttling
        if (SLOT0_I2C_PERIOD_MS > 0 && (nowMs - slot0LastI2cMs) < SLOT0_I2C_PERIOD_MS) {
            return slot0Frame; // keep "no new info" behavior; you can change to last-known if you want
        }
        slot0LastI2cMs = nowMs;

        // Distance reads ONCE
        double d1 = slot0Color1.getDistance(DistanceUnit.CM);
        double d2 = slot0Color2.getDistance(DistanceUnit.CM);

        boolean p1 = !Double.isNaN(d1) && d1 <= SLOT0_DIST_THRESH_CM;
        boolean p2 = !Double.isNaN(d2) && d2 <= SLOT0_DIST_THRESH_CM;

        boolean present = p1 || p2;
        if (!present) return slot0Frame;

        RevColorSensorV3 chosen;
        if (p1 && p2) chosen = (d1 <= d2) ? slot0Color1 : slot0Color2;
        else chosen = p1 ? slot0Color1 : slot0Color2;

        RawColor raw = classifyColor(chosen);
        Ball c = (raw == RawColor.GREEN) ? Ball.GREEN
                : (raw == RawColor.PURPLE) ? Ball.PURPLE
                : Ball.UNKNOWN;

        slot0Frame.present = true;
        slot0Frame.color = c;
        return slot0Frame;
    }

    private RawColor classifyColor(RevColorSensorV3 sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int sum = r + g + b;
        if (sum < 50) return RawColor.RED;

        double rn = r / (double) sum;
        double gn = g / (double) sum;
        double bn = b / (double) sum;

        boolean greenDominant = (gn > rn + 0.05) && (gn > bn + 0.02);
        boolean redDominant   = (rn > gn + 0.10) && (rn > bn + 0.05);

        if (greenDominant) return RawColor.GREEN;
        if (redDominant) return RawColor.RED;
        return RawColor.PURPLE;
    }

    private boolean isAlignedAtIntake() {
        double cur = getCurrentAngleDeg();
        double err = smallestAngleDiff(normalizeAngle(INTAKE_ANGLE_DEG), cur);
        return Math.abs(err) <= ALIGN_TOL_DEG;
    }

    // =========================
    // ===== MOTION CONTROL =====
    // =========================
    private void setMotorPower(double pwr) {
        dbgLastRequestedPower = pwr;

        double out = REVERSE_MOTOR ? -pwr : pwr;
        out = Range.clip(out, -1.0, 1.0);

        dbgLastAppliedPower = out;
        motor.setPower(out);
    }

    private double computeErrorDeg(double targetDeg, ErrorMode mode) {
        double cur = getCurrentAngleDeg();
        double target = normalizeAngle(targetDeg);

        double eShort = smallestAngleDiff(target, cur);
        dbgLastErrShortestDeg = eShort;

        if (mode == ErrorMode.SHORTEST) return eShort;

        if (mode == ErrorMode.PREFER_CW) {
            if (eShort < -OVERSHOOT_CORRECT_MAX_DEG) return eShort + 360.0;
            return eShort;
        }

        if (eShort > OVERSHOOT_CORRECT_MAX_DEG) return eShort - 360.0;
        return eShort;
    }

    private boolean driveToTarget(double targetDeg, ErrorMode mode, double maxPower) {
        double cur = getCurrentAngleDeg();
        double target = normalizeAngle(targetDeg);

        double eShort = smallestAngleDiff(target, cur);
        if (Math.abs(eShort) <= MOVE_DONE_TOL_DEG) {
            targetAngleDeg = target;
            setMotorPower(0.0);
            resetDrivePID();
            dbgLastErrUsedDeg = 0.0;
            return true;
        }

        double e = computeErrorDeg(target, mode);
        dbgLastErrUsedDeg = e;

        long nowNs = System.nanoTime();
        double dt = (nowNs - pidLastNs) / 1e9;
        if (dt <= 0) dt = 1e-3;
        pidLastNs = nowNs;

        pidIntegral += e * dt;
        pidIntegral = Range.clip(pidIntegral, -I_CLAMP, I_CLAMP);

        double deriv = (e - pidLastErrDeg) / dt;
        pidLastErrDeg = e;

        double ff = kF * Math.signum(e);

        double out = kP * e + kI * pidIntegral + kD * deriv + ff;
        out = Range.clip(out, -maxPower, maxPower);

        targetAngleDeg = target;
        setMotorPower(out);
        return false;
    }

    private boolean moveTowardTargetCW(double targetDeg) {
        return driveToTarget(targetDeg, ErrorMode.PREFER_CW, MOVE_MAX_POWER);
    }

    private boolean moveTowardTargetCCW(double targetDeg) {
        return driveToTarget(targetDeg, ErrorMode.PREFER_CCW, MOVE_MAX_POWER);
    }

    private void holdToTargetShortestPath() {
        driveToTarget(targetAngleDeg, ErrorMode.SHORTEST, HOLD_MAX_POWER);
    }

    // =========================
    // ===== SHOOTING ===========
    // =========================
    private void beginShooting(long nowMs) {
        shootRequested = false;
        shootStartMs = nowMs;
        shootMovedDeg = 0.0;
        shootLastTicks = getTicksCached();   // cached ticks (no extra hub read)
    }

    private boolean updateShooting(long nowMs) {
        if (nowMs - shootStartMs > SHOOT_TIMEOUT_MS) {
            setMotorPower(0.0);
            return true;
        }

        int curTicks = getTicksCached();     // cached ticks (no extra hub read)
        int dTicks = curTicks - shootLastTicks;
        shootLastTicks = curTicks;

        shootMovedDeg += Math.abs(dTicks) * DEG_PER_TICK;

        double pwr = (nowMs - shootStartMs <= SHOOT_HIGH_MS) ? SHOOT_POWER_HIGH : SHOOT_POWER_LOW;

        // CCW eject => negative power in angle-space
        setMotorPower(-pwr);

        if (shootMovedDeg >= SHOOT_DEG) {
            setMotorPower(0.0);
            return true;
        }

        return false;
    }

    // =========================
    // ===== ANGLE MATH =========
    // =========================
    private double normalizeAngle(double a) {
        return (a % 360.0 + 360.0) % 360.0;
    }

    private double smallestAngleDiff(double target, double current) {
        double diff = normalizeAngle(target) - normalizeAngle(current);
        diff = (diff + 540.0) % 360.0 - 180.0;
        return diff;
    }

    private double cwDeltaDeg(double fromDeg, double toDeg) {
        double f = normalizeAngle(fromDeg);
        double t = normalizeAngle(toDeg);
        return (t - f + 360.0) % 360.0;
    }

    private double ticksToAngle(int ticks) {
        int rel = ticks - zeroTicks;
        int signedRel = REVERSE_MOTOR ? -rel : rel;

        double revs = signedRel / TICKS_PER_REV;
        return normalizeAngle(INTAKE_ANGLE_DEG + revs * 360.0);
    }

    private int angleToTicks(double angleDeg) {
        double norm = normalizeAngle(angleDeg - INTAKE_ANGLE_DEG);
        double revs = norm / 360.0;

        int signedRel = (int) Math.round(revs * TICKS_PER_REV);
        int rel = REVERSE_MOTOR ? -signedRel : signedRel;
        return zeroTicks + rel;
    }

    private static RevColorSensorV3 safeGetRev(HardwareMap hw, String name) {
        try {
            return hw.get(RevColorSensorV3.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
