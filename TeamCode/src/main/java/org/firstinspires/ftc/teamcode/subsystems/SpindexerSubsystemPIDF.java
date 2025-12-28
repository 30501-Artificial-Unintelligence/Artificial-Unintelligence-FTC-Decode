package org.firstinspires.ftc.teamcode.subsystems;

// If you use the older configurable annotation (what you used earlier):
import com.bylazar.configurables.annotations.Configurable;

// If you use FTControl Panels' annotation instead, use this and remove the line above:


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class SpindexerSubsystemPIDF {

    // =========================
    // Tunable params (Panels)
    // =========================
    public static class Tune {
        // Motion
        public static double MOVE_POWER = 0.50;
        public static int    MOVE_TIMEOUT_MS = 700;

        // Auto-rotate after detecting a ball (increase to keep it at intake longer)
        public static long AUTO_ROTATE_DELAY_MS = 180;

        // Shooter/eject timing
        public static long WAIT_BEFORE_LOADER_FULL_MS    = 200;
        public static long WAIT_BEFORE_LOADER_PARTIAL_MS = 2000;
        public static long WAIT_AFTER_LOADER_MS          = 400;

        // Absolute encoder calibration:
        // absInternalDeg = normalize(absRawDeg - ABS_MECH_OFFSET_DEG)
        public static double ABS_MECH_OFFSET_DEG = 245.0;

        // ABS sanity / rezero behavior
        public static boolean ABS_ENABLE_SANITY = true;

        // if |enc - abs| exceeds this for N consecutive checks, rezero from ABS
        public static double ABS_REZERO_THRESHOLD_DEG = 3.0;

        // require this many consecutive "bad" checks before rezero
        public static int ABS_REZERO_STREAK = 3;

        // don't rezero more frequently than this
        public static long ABS_REZERO_COOLDOWN_MS = 600;

        // ignore ABS readings that "jump" too much from last reading
        public static double ABS_MAX_JUMP_DEG = 25.0;

        // voltage health (ignore if abs wire/glitch)
        public static double ABS_MIN_VOLT = 0.05;
        public static double ABS_MAX_VOLT = 3.25;

        // use N-sample circular mean for ABS angle to reduce noise
        public static int ABS_AVG_SAMPLES = 5;

        // Optional: after a blocking move, if ABS says we're off target, do a low-power nudge
        public static boolean ABS_POSTMOVE_NUDGE = true;
        public static double  ABS_POSTMOVE_TOL_DEG = 2.0;
        public static double  ABS_POSTMOVE_POWER = 0.25;
        public static int     ABS_POSTMOVE_TIMEOUT_MS = 350;

        // Motor PIDF (RUN_TO_POSITION) - FTC built-in controller
        public static double kP = 10.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }

    // =========================
    // Fixed constants
    // =========================
    private static final double TICKS_PER_REV = 383.6;      // goBILDA 435 RPM YJ integrated encoder
    private static final double DEGREES_PER_SLOT = 120.0;
    private static final double INTAKE_ANGLE = 0.0;
    private static final double LOAD_ANGLE   = 180.0;
    private static final int    SLOT_COUNT   = 3;
    private static final int    TOLERANCE_TICKS = 10;

    private static final double ABS_VREF = 3.3;

    // =========================
    // Types / state
    // =========================
    public enum Ball { EMPTY, GREEN, PURPLE, UNKNOWN }
    private enum RawColor { RED, GREEN, PURPLE }

    private final DcMotorEx motor;
    private final RevColorSensorV3 intakeColor1;
    private final RevColorSensorV3 intakeColor2;
    private final AnalogInput absEncoder;

    private final Ball[] slots = new Ball[SLOT_COUNT];

    // encoder alignment: internalAngleDeg = (ticks - zeroTicks) * 360 / TICKS_PER_REV
    private int zeroTicks = 0;

    // which slot index is currently at intake (0,1,2)
    private int intakeIndex = 0;

    // Auto intake state
    private boolean lastBallPresent = false;
    private boolean pendingAutoRotate = false;
    private long autoRotateTimeMs = 0;

    // Eject state
    private boolean ejecting = false;
    private int ejectSlotIndex = 0;
    private long ejectPhaseTime = 0;
    private int ejectPhase = 0; // 0=rotate,1=wait before loader,2=wait after loader
    private boolean startedWithFullMag = false;

    // Pattern state
    private int gameTag = 0;
    private int patternStep = 0;

    // ABS sanity tracking
    private long lastAbsRezeroMs = 0;
    private int absBadStreak = 0;
    private double lastAbsInternalDeg = 0.0;
    private boolean hasLastAbs = false;

    // PIDF caching
    private double lastP = Double.NaN, lastI = Double.NaN, lastD = Double.NaN, lastF = Double.NaN;

    // =========================
    // Constructor
    // =========================
    public SpindexerSubsystemPIDF(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        intakeColor1 = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor1");
        intakeColor2 = hardwareMap.get(RevColorSensorV3.class, "spindexerIntakeColorSensor2");
        absEncoder   = hardwareMap.get(AnalogInput.class, "spindexerAbs");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for (int i = 0; i < SLOT_COUNT; i++) slots[i] = Ball.EMPTY;

        applyRunToPositionPidfIfChanged();

        // One-shot alignment from ABS at boot
        Double absInternal = readAbsInternalDegAveraged();
        if (absInternal != null) {
            autoZeroFromAbsInternal(absInternal);
            lastAbsInternalDeg = absInternal;
            hasLastAbs = true;
        }

        homeToIntake();
    }

    // =========================
    // Public helpers
    // =========================
    public void setGameTag(int tag) { gameTag = tag; }
    public int getGameTag() { return gameTag; }

    public Ball[] getSlots() { return slots; }
    public boolean isFull() { for (Ball b : slots) if (b == Ball.EMPTY) return false; return true; }
    public boolean hasAnyBall() { for (Ball b : slots) if (b != Ball.EMPTY) return true; return false; }
    public boolean slotHasBall(int idx) { return slots[idx] != Ball.EMPTY; }
    public void clearSlot(int idx) { slots[idx] = Ball.EMPTY; }
    public int getIntakeSlotIndex() { return intakeIndex; }
    public boolean isEjecting() { return ejecting; }
    public boolean isAutoRotating() { return pendingAutoRotate; }

    public int getEncoderTicks() { return motor.getCurrentPosition(); }
    public int getTargetTicks() { return motor.getTargetPosition(); }
    public int getZeroTicks() { return zeroTicks; }

    /** Encoder-based internal angle in [0,360). */
    public double getCurrentAngleDeg() {
        return ticksToInternalAngleDeg(motor.getCurrentPosition());
    }

    /** ABS internal angle in [0,360), averaged; null if unhealthy. */
    public Double getAbsInternalAngleDeg() {
        return readAbsInternalDegAveraged();
    }

    /** Difference (enc - abs) in degrees [-180,180); null if ABS unhealthy. */
    public Double getEncMinusAbsDeg() {
        Double abs = readAbsInternalDegAveraged();
        if (abs == null) return null;
        return smallestAngleDiff(getCurrentAngleDeg(), abs);
    }

    public void homeToIntake() {
        moveSlotToIntake(0, Tune.MOVE_POWER);
    }

    // =========================
    // Main update
    // =========================
    public boolean update(Telemetry telemetry,
                          LoaderSubsystem loader,
                          boolean yEdge,
                          int patternTagOverride) {

        applyRunToPositionPidfIfChanged();

        // --- Optional manual override for tag ---
        if (patternTagOverride == 21 ||
                patternTagOverride == 22 ||
                patternTagOverride == 23 ||
                patternTagOverride == 0) {
            gameTag = patternTagOverride;
        }

        // --- Start eject sequence on Y rising edge ---
        if (yEdge && !ejecting) {
            if (hasAnyBall()) {
                ejecting = true;
                ejectPhase = 0;
                patternStep = 0;
                startedWithFullMag = isFull();
            } else {
                // "force try shoot 3" fallback
                Ball[] pattern = getPatternForTag(gameTag);
                if (pattern != null) {
                    for (int i = 0; i < SLOT_COUNT; i++) slots[i] = pattern[i];
                } else {
                    for (int i = 0; i < SLOT_COUNT; i++) slots[i] = Ball.PURPLE;
                }
                ejecting = true;
                ejectPhase = 0;
                patternStep = 0;
                startedWithFullMag = false;
            }
        }

        // --- Auto-intake detect ---
        if (!isFull()) {
            boolean ballPresent = isBallPresent();
            if (ballPresent && !lastBallPresent && slots[intakeIndex] == Ball.EMPTY) {
                Ball color = detectBallColor();
                if (color == Ball.GREEN || color == Ball.PURPLE) {
                    slots[intakeIndex] = color;

                    pendingAutoRotate = true;
                    autoRotateTimeMs = System.currentTimeMillis() + Tune.AUTO_ROTATE_DELAY_MS;
                }
            }
            lastBallPresent = ballPresent;
        }

        // --- Pending auto-rotate ---
        if (pendingAutoRotate && System.currentTimeMillis() >= autoRotateTimeMs) {
            if (!motor.isBusy() && !ejecting) {
                pendingAutoRotate = false;
                int nextIndex = (int) ((intakeIndex + 1) % SLOT_COUNT);
                startRunToAngleAsync(slotCenterAngleAtIntake(nextIndex), Tune.MOVE_POWER);
                intakeIndex = nextIndex;
            }
        }

        // --- Eject state machine ---
        if (ejecting) {
            long now = System.currentTimeMillis();
            switch (ejectPhase) {
                case 0: {
                    if (!hasAnyBall()) {
                        homeToIntake();
                        ejecting = false;
                        startedWithFullMag = false;
                        break;
                    }

                    int nextSlot = selectNextEjectSlotIndex();
                    if (nextSlot < 0 || nextSlot >= SLOT_COUNT) {
                        homeToIntake();
                        ejecting = false;
                        startedWithFullMag = false;
                        break;
                    }

                    ejectSlotIndex = nextSlot;

                    // rotate to LOAD (blocking)
                    moveSlotToLoadBlocking(ejectSlotIndex);

                    long delayMs = startedWithFullMag
                            ? Tune.WAIT_BEFORE_LOADER_FULL_MS
                            : Tune.WAIT_BEFORE_LOADER_PARTIAL_MS;

                    ejectPhaseTime = now + delayMs;
                    ejectPhase = 1;
                    break;
                }

                case 1:
                    if (now >= ejectPhaseTime) {
                        loader.startCycle();
                        ejectPhaseTime = now + Tune.WAIT_AFTER_LOADER_MS;
                        ejectPhase = 2;
                    }
                    break;

                case 2:
                    if (now >= ejectPhaseTime) {
                        clearSlot(ejectSlotIndex);

                        Ball[] pattern = getPatternForTag(gameTag);
                        if (pattern != null) patternStep++;

                        boolean usingPattern = (pattern != null);
                        if (!hasAnyBall() || (usingPattern && patternStep >= pattern.length)) {
                            homeToIntake();
                            ejecting = false;
                            startedWithFullMag = false;
                        } else {
                            ejectPhase = 0;
                        }
                    }
                    break;
            }
        }

        // --- ABS sanity check when idle ---
        if (Tune.ABS_ENABLE_SANITY) {
            boolean safeToCheck = !motor.isBusy() && !ejecting && !pendingAutoRotate;
            if (safeToCheck) {
                maybeResyncFromAbs();
            } else {
                absBadStreak = 0;
            }
        }

        // Optional debug
        if (telemetry != null) {
            Double abs = getAbsInternalAngleDeg();
            Double diff = getEncMinusAbsDeg();
            telemetry.addData("Spd encDeg", "%.1f", getCurrentAngleDeg());
            telemetry.addData("Spd absDeg", abs == null ? "null" : String.format("%.1f", abs));
            telemetry.addData("Spd enc-abs", diff == null ? "null" : String.format("%.1f", diff));
            telemetry.addData("Spd idx", intakeIndex);
            telemetry.addData("Spd zeroTicks", zeroTicks);
        }

        return isFull();
    }

    // =========================
    // Motion / angles
    // =========================
    private double normalizeAngle(double angleDeg) {
        return (angleDeg % 360.0 + 360.0) % 360.0;
    }

    /** a-b wrapped to [-180,180) */
    private double smallestAngleDiff(double a, double b) {
        double diff = a - b;
        diff = (diff + 540.0) % 360.0 - 180.0;
        return diff;
    }

    private double ticksToInternalAngleDeg(int ticks) {
        int relTicks = ticks - zeroTicks;
        double revs = relTicks / TICKS_PER_REV;
        return normalizeAngle(revs * 360.0);
    }

    private int internalAngleDegToTicksNearest(double angleDeg) {
        double norm = normalizeAngle(angleDeg);

        double desiredRevs = norm / 360.0;
        int baseTicks = zeroTicks + (int) Math.round(desiredRevs * TICKS_PER_REV);

        int current = motor.getCurrentPosition();
        int revTicks = (int) Math.round(TICKS_PER_REV);

        int diff = baseTicks - current;
        int k = (int) Math.round((double) diff / revTicks);

        return baseTicks - k * revTicks;
    }

    private void startRunToAngleAsync(double angleDeg, double power) {
        int target = internalAngleDegToTicksNearest(angleDeg);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private void runToAngleBlocking(double angleDeg, double power, boolean allowPostNudge) {
        applyRunToPositionPidfIfChanged();

        int target = internalAngleDegToTicksNearest(angleDeg);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        long start = System.currentTimeMillis();
        while (motor.isBusy() && (System.currentTimeMillis() - start) < Tune.MOVE_TIMEOUT_MS) {
            try { Thread.sleep(5); } catch (InterruptedException ignored) { break; }
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (Tune.ABS_ENABLE_SANITY) {
            // After move, optionally snap zero if we're drifting
            maybeResyncFromAbs();

            // Optional: nudge to target based on ABS if still off target
            if (allowPostNudge && Tune.ABS_POSTMOVE_NUDGE) {
                Double absInternal = readAbsInternalDegAveraged();
                if (absInternal != null) {
                    double targetNorm = normalizeAngle(angleDeg);
                    double absErrToTarget = smallestAngleDiff(absInternal, targetNorm);

                    if (Math.abs(absErrToTarget) > Tune.ABS_POSTMOVE_TOL_DEG) {
                        // Recompute zeroTicks from ABS and do one low-power short nudge
                        autoZeroFromAbsInternal(absInternal);

                        int target2 = internalAngleDegToTicksNearest(angleDeg);
                        motor.setTargetPosition(target2);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor.setPower(Tune.ABS_POSTMOVE_POWER);

                        long start2 = System.currentTimeMillis();
                        while (motor.isBusy() && (System.currentTimeMillis() - start2) < Tune.ABS_POSTMOVE_TIMEOUT_MS) {
                            try { Thread.sleep(5); } catch (InterruptedException ignored) { break; }
                        }
                        motor.setPower(0);
                        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }
        }
    }

    public void moveToAngleBlocking(double angleDeg) {
        runToAngleBlocking(angleDeg, Tune.MOVE_POWER, true);
    }

    public void moveToAngleBlocking(double angleDeg, double power) {
        runToAngleBlocking(angleDeg, power, true);
    }

    public void moveToAngleAsync(double angleDeg, double power) {
        startRunToAngleAsync(angleDeg, power);
    }

    private double slotCenterAngleAtIntake(int slotIndex) {
        return INTAKE_ANGLE + slotIndex * DEGREES_PER_SLOT;
    }

    private void moveSlotToIntake(int slotIndex, double power) {
        runToAngleBlocking(slotCenterAngleAtIntake(slotIndex), power, true);
        intakeIndex = slotIndex;
    }

    private void moveSlotToLoadBlocking(int slotIndex) {
        double angle = slotCenterAngleAtIntake(slotIndex) + (LOAD_ANGLE - INTAKE_ANGLE);
        runToAngleBlocking(angle, Tune.MOVE_POWER, true);
    }

    // =========================
    // ABS read + sanity
    // =========================
    private boolean absVoltageHealthy(double v) {
        return v >= Tune.ABS_MIN_VOLT && v <= Tune.ABS_MAX_VOLT;
    }

    private double getAbsRawDeg() {
        double v = absEncoder.getVoltage();
        double angle = (v / ABS_VREF) * 360.0;
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    /** single-sample internal angle deg in [0,360), or null if voltage unhealthy */
    private Double readAbsInternalDegOnce() {
        double v = absEncoder.getVoltage();
        if (!absVoltageHealthy(v)) return null;
        double raw = (v / ABS_VREF) * 360.0;
        raw = normalizeAngle(raw);
        return normalizeAngle(raw - Tune.ABS_MECH_OFFSET_DEG);
    }

    /** circular-mean averaged internal angle in [0,360), or null if unhealthy */
    private Double readAbsInternalDegAveraged() {
        int n = Math.max(1, Tune.ABS_AVG_SAMPLES);

        double sumSin = 0.0;
        double sumCos = 0.0;
        int good = 0;

        for (int i = 0; i < n; i++) {
            Double a = readAbsInternalDegOnce();
            if (a == null) continue;
            double rad = Math.toRadians(a);
            sumSin += Math.sin(rad);
            sumCos += Math.cos(rad);
            good++;
        }

        if (good == 0) return null;

        double avgRad = Math.atan2(sumSin / good, sumCos / good);
        if (avgRad < 0) avgRad += 2.0 * Math.PI;
        return Math.toDegrees(avgRad);
    }

    /** Snap zeroTicks + intakeIndex from a known-good ABS internal angle. */
    private void autoZeroFromAbsInternal(double internalAngleDeg0to360) {
        double internal = normalizeAngle(internalAngleDeg0to360);
        int currentTicks = motor.getCurrentPosition();

        int internalTicks = (int) Math.round((internal / 360.0) * TICKS_PER_REV);
        zeroTicks = currentTicks - internalTicks;

        // update intakeIndex estimate from ABS
        double slotIndexF = internal / DEGREES_PER_SLOT;
        int nearestIndex = (int) Math.round(slotIndexF) % SLOT_COUNT;
        if (nearestIndex < 0) nearestIndex += SLOT_COUNT;
        intakeIndex = nearestIndex;
    }

    /** Called only when safe/idle. Uses streak + cooldown to avoid noise re-zero spam. */
    private void maybeResyncFromAbs() {
        long now = System.currentTimeMillis();
        if (now - lastAbsRezeroMs < Tune.ABS_REZERO_COOLDOWN_MS) return;

        Double absInternal = readAbsInternalDegAveraged();
        if (absInternal == null) { absBadStreak = 0; return; }

        if (hasLastAbs) {
            double jump = Math.abs(smallestAngleDiff(absInternal, lastAbsInternalDeg));
            if (jump > Tune.ABS_MAX_JUMP_DEG) {
                // ABS glitched/jumped; ignore this sample
                absBadStreak = 0;
                lastAbsInternalDeg = absInternal;
                return;
            }
        }

        lastAbsInternalDeg = absInternal;
        hasLastAbs = true;

        double encInternal = getCurrentAngleDeg();
        double diff = smallestAngleDiff(encInternal, absInternal);

        if (Math.abs(diff) > Tune.ABS_REZERO_THRESHOLD_DEG) {
            absBadStreak++;
        } else {
            absBadStreak = 0;
        }

        if (absBadStreak >= Tune.ABS_REZERO_STREAK) {
            autoZeroFromAbsInternal(absInternal);
            lastAbsRezeroMs = now;
            absBadStreak = 0;
        }
    }

    // =========================
    // Motor PIDF (RUN_TO_POSITION)
    // =========================
    private void applyRunToPositionPidfIfChanged() {
        if (Tune.kP == lastP && Tune.kI == lastI && Tune.kD == lastD && Tune.kF == lastF) return;

        try {
            PIDFCoefficients pidf = new PIDFCoefficients(Tune.kP, Tune.kI, Tune.kD, Tune.kF);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);
            lastP = Tune.kP; lastI = Tune.kI; lastD = Tune.kD; lastF = Tune.kF;
        } catch (Exception ignored) {
            // Some configs/SDK combos may not support this call; safe to ignore
        }
    }

    // =========================
    // Ball sensing
    // =========================
    private boolean isBallPresent() {
        final double THRESH_CM = 5.0;
        double d1 = intakeColor1.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean p1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean p2 = !Double.isNaN(d2) && d2 <= THRESH_CM;
        return p1 || p2;
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

    private Ball detectBallColor() {
        final double THRESH_CM = 5.0;

        double d1 = intakeColor1.getDistance(DistanceUnit.CM);
        double d2 = intakeColor2.getDistance(DistanceUnit.CM);

        boolean p1 = !Double.isNaN(d1) && d1 <= THRESH_CM;
        boolean p2 = !Double.isNaN(d2) && d2 <= THRESH_CM;

        if (!p1 && !p2) return Ball.EMPTY;

        RevColorSensorV3 chosen = (p1 && p2) ? ((d1 <= d2) ? intakeColor1 : intakeColor2)
                : (p1 ? intakeColor1 : intakeColor2);

        RawColor raw = classifyColor(chosen);
        if (raw == RawColor.GREEN)  return Ball.GREEN;
        if (raw == RawColor.PURPLE) return Ball.PURPLE;
        return Ball.EMPTY; // RED treated as non-ball
    }

    // =========================
    // Pattern logic
    // =========================
    private Ball[] getPatternForTag(int tag) {
        Ball[] seq = new Ball[SLOT_COUNT];
        switch (tag) {
            case 23: seq[0]=Ball.PURPLE; seq[1]=Ball.PURPLE; seq[2]=Ball.GREEN;  return seq;
            case 22: seq[0]=Ball.PURPLE; seq[1]=Ball.GREEN;  seq[2]=Ball.PURPLE; return seq;
            case 21: seq[0]=Ball.GREEN;  seq[1]=Ball.PURPLE; seq[2]=Ball.PURPLE; return seq;
            default: return null;
        }
    }

    private int selectFastestNonEmptySlot(double currentAngle) {
        int bestSlot = -1;
        double bestDiff = Double.MAX_VALUE;

        for (int i = 0; i < SLOT_COUNT; i++) {
            if (!slotHasBall(i)) continue;

            double targetAngle = slotCenterAngleAtIntake(i) + (LOAD_ANGLE - INTAKE_ANGLE);
            double diff = Math.abs(smallestAngleDiff(targetAngle, currentAngle));

            if (diff < bestDiff) {
                bestDiff = diff;
                bestSlot = i;
            }
        }
        return bestSlot;
    }

    private int selectNextEjectSlotIndex() {
        double currentAngle = getCurrentAngleDeg();
        Ball[] pattern = getPatternForTag(gameTag);
        boolean usePattern = (pattern != null && patternStep < pattern.length);

        if (usePattern) {
            Ball desired = pattern[patternStep];

            int bestSlot = -1;
            double bestDiff = Double.MAX_VALUE;

            for (int i = 0; i < SLOT_COUNT; i++) {
                if (slots[i] != desired) continue;

                double targetAngle = slotCenterAngleAtIntake(i) + (LOAD_ANGLE - INTAKE_ANGLE);
                double diff = Math.abs(smallestAngleDiff(targetAngle, currentAngle));

                if (diff < bestDiff) {
                    bestDiff = diff;
                    bestSlot = i;
                }
            }
            if (bestSlot != -1) return bestSlot;
        }

        return selectFastestNonEmptySlot(currentAngle);
    }

    public void rezeroFromAbsNow() {
        Double absInternal = readAbsInternalDegAveraged();
        if (absInternal == null) return;

        autoZeroFromAbsInternal(absInternal);

        lastAbsInternalDeg = absInternal;
        hasLastAbs = true;
        lastAbsRezeroMs = System.currentTimeMillis();
        absBadStreak = 0;
    }

}
