package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class TurretSubsystemIncremental {

    // =========================
    // Motor / gear constants
    // =========================
    // goBILDA 435 rpm is ~383.6 ticks per output rev (you can keep 384.5 if that matches your testing)
    public static double TICKS_PER_MOTOR_REV = 383.6;

    // 60T motor pulley -> 280T turret pulley
    public static double MOTOR_REV_PER_TURRET_REV = 280.0 / 60.0;

    public static double TICKS_PER_TURRET_REV =
            TICKS_PER_MOTOR_REV * MOTOR_REV_PER_TURRET_REV;

    public static double TICKS_PER_TURRET_DEG = TICKS_PER_TURRET_REV / 360.0;

    // Mechanical window
    public static double MIN_ANGLE_DEG = -200.0;
    public static double MAX_ANGLE_DEG =  200.0;

    // Same idea as your MAX_AUTO_POWER, but tunable from dashboard
    public static double MAX_AUTO_POWER = 0.85;

    // Position tolerance (ticks) used by RUN_TO_POSITION
    public static double TARGET_TOL_DEG = 0.8; // 0.6–1.5 deg typical
    public static int TARGET_TOL_TICKS = 6;    // overwritten in applyTuning()

    // Optional: cap motor velocity for consistency (ticks/sec)
    // goBILDA 435 rpm => ~7.25 rev/s => ~7.25*383.6 ≈ 2780 ticks/s free-ish
    public static boolean USE_VELOCITY_CAP = true;
    public static double MAX_VEL_TICKS_PER_SEC = 2400.0;

    // Built-in motor PID tuning (this is often the #1 reason RUN_TO_POSITION feels slow)
    // These affect RUN_USING_ENCODER velocity control and thus RUN_TO_POSITION behavior.
    public static double VEL_P = 10.0;
    public static double VEL_I = 3.0;
    public static double VEL_D = 0.0;
    public static double VEL_F = 0.0;

    // RUN_TO_POSITION position P (bigger = drives harder toward target)
    // If you get oscillation/overshoot, reduce this.
    public static double POS_P = 12.0; // start 8..20 for turrets

    // Small deadband so we don’t spam new targets when dx/dy are tiny
    public static double FACE_TARGET_MIN_DIST_IN = 0.5;

    // Turret mount offset if turret 0 is not perfectly robot-forward
    public static double TURRET_MOUNT_OFFSET_DEG = 0.0;

    // =========================
    // Hardware
    // =========================
    private final DcMotorEx turretMotor;

    // =========================
    // Incremental reference
    // currentAngleDeg = angleOffsetDeg + ticks / TICKS_PER_TURRET_DEG
    // =========================
    private double angleOffsetDeg = 0.0;

    // Control state
    private double targetAngleDeg = 0.0;
    private boolean trackEnabled = false;

    public TurretSubsystemIncremental(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // We keep your approach: start in RUN_USING_ENCODER; goToAngle switches to RUN_TO_POSITION.
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply built-in controller tuning (big improvement for “slow RUN_TO_POSITION”)
        applyTuning();

        // Restore offset from previous OpMode if available
        if (OpModeStorage.turretAngleOffsetDeg != null) {
            angleOffsetDeg = OpModeStorage.turretAngleOffsetDeg;
        } else {
            // Default: define "current position is 0°" until you call homeHereAsZero()
            angleOffsetDeg = - (turretMotor.getCurrentPosition() / TICKS_PER_TURRET_DEG);
        }

        if (OpModeStorage.turretTrackEnabled != null) {
            trackEnabled = OpModeStorage.turretTrackEnabled;
        }

        targetAngleDeg = getCurrentAngleDeg();
    }

    // =========================
    // Tuning helpers
    // =========================
    private void applyTuning() {
        // Update derived constants (in case you tune ticks/rev live from dashboard)
        TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * MOTOR_REV_PER_TURRET_REV;
        TICKS_PER_TURRET_DEG = TICKS_PER_TURRET_REV / 360.0;

        TARGET_TOL_TICKS = (int) Math.max(1, Math.round(TARGET_TOL_DEG * TICKS_PER_TURRET_DEG));
        turretMotor.setTargetPositionTolerance(TARGET_TOL_TICKS);

        // These methods exist on most FTC SDK versions with DcMotorEx
        // If your SDK complains, tell me the compile error and I’ll swap to the compatible call.
        turretMotor.setVelocityPIDFCoefficients(VEL_P, VEL_I, VEL_D, VEL_F);
        turretMotor.setPositionPIDFCoefficients(POS_P);
    }

    private void setModeIfNeeded(DcMotor.RunMode mode) {
        if (turretMotor.getMode() != mode) {
            turretMotor.setMode(mode);
        }
    }

    // =========================
    // Reference / homing
    // =========================

    /** Call this once after you manually aim turret straight forward. */
    public void homeHereAsZero() {
        setAngleReferenceDeg(0.0);
    }

    /**
     * Set what the current turret angle *should be* without resetting encoders.
     * Example: setAngleReferenceDeg(0) when turret is physically forward.
     */
    public void setAngleReferenceDeg(double currentAngleShouldBeDeg) {
        int ticks = turretMotor.getCurrentPosition();
        double tickAngle = ticks / TICKS_PER_TURRET_DEG;
        angleOffsetDeg = currentAngleShouldBeDeg - tickAngle;

        // persist for next OpMode
        OpModeStorage.turretAngleOffsetDeg = angleOffsetDeg;
    }

    // =========================
    // Tick / angle helpers
    // =========================

    public double getCurrentAngleDeg() {
        int ticks = turretMotor.getCurrentPosition();
        return angleOffsetDeg + (ticks / TICKS_PER_TURRET_DEG);
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    private int turretDegToTicks(double turretDeg) {
        return (int) Math.round((turretDeg - angleOffsetDeg) * TICKS_PER_TURRET_DEG);
    }

    // Wrap degrees to [-180, 180)
    private static double wrapDeg180(double a) {
        while (a >= 180.0) a -= 360.0;
        while (a <  -180.0) a += 360.0;
        return a;
    }

    /**
     * Choose best equivalent angle (angle + 360*k) within [MIN, MAX], minimizing motion.
     */
    private double chooseWrappedWithinLimits(double desiredDeg, double currentDeg) {
        double best = Double.NaN;
        double bestCost = Double.POSITIVE_INFINITY;

        for (int k = -2; k <= 2; k++) {
            double cand = desiredDeg + 360.0 * k;
            if (cand < MIN_ANGLE_DEG || cand > MAX_ANGLE_DEG) continue;

            double cost = Math.abs(cand - currentDeg);
            if (cost < bestCost) {
                bestCost = cost;
                best = cand;
            }
        }

        if (!Double.isNaN(best)) return best;
        return Range.clip(desiredDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    // =========================
    // Manual + position control
    // =========================

    public void setManualPower(double power) {
        double current = getCurrentAngleDeg();
        double marginDeg = 5.0;

        if (power > 0 && current >= MAX_ANGLE_DEG - marginDeg) power = 0.0;
        else if (power < 0 && current <= MIN_ANGLE_DEG + marginDeg) power = 0.0;

        power = Range.clip(power, -1.0, 1.0);

        // If we were in RUN_TO_POSITION, switch back once (don’t spam)
        setModeIfNeeded(DcMotor.RunMode.RUN_USING_ENCODER);

        turretMotor.setPower(power);
    }

    public void goToAngle(double commandedAngleDeg) {
        // Re-apply tuning occasionally if you tweak dashboard constants live
        // (cheap and keeps behavior consistent)
        applyTuning();

        double currentDeg = getCurrentAngleDeg();
        targetAngleDeg = chooseWrappedWithinLimits(commandedAngleDeg, currentDeg);

        int targetTicks = turretDegToTicks(targetAngleDeg);

        turretMotor.setTargetPosition(targetTicks);

        // Mode spam makes things feel sluggish; only set if needed.
        setModeIfNeeded(DcMotor.RunMode.RUN_TO_POSITION);

        // Optional: velocity cap makes it reach target faster *consistently* without weird ramping
        if (USE_VELOCITY_CAP) {
            turretMotor.setVelocity(MAX_VEL_TICKS_PER_SEC);
        }

        turretMotor.setPower(MAX_AUTO_POWER);
    }

    // =========================
    // Tracking helpers
    // =========================
    public void setTrackEnabled(boolean enabled) {
        trackEnabled = enabled;
        OpModeStorage.turretTrackEnabled = enabled;
    }
    public boolean isTrackEnabled() { return trackEnabled; }

    public static double VISION_DEADBAND_DEG = 0.15;
    public static double VISION_GAIN = 3.0;

    // If this is huge, you “teleport” the target and RUN_TO_POSITION looks slow because you keep retargeting.
    public static double VISION_MAX_STEP_DEG = 10.0; // try 6..12 for smooth tracking

    public void trackWithTxDeg(double txDeg) {
        if (!trackEnabled) return;
        if (Double.isNaN(txDeg)) return;
        if (Math.abs(txDeg) < VISION_DEADBAND_DEG) return;

        double stepDeg = Range.clip(txDeg * VISION_GAIN, -VISION_MAX_STEP_DEG, VISION_MAX_STEP_DEG);
        double newTarget = getCurrentAngleDeg() + stepDeg;
        goToAngle(newTarget);
    }

    /**
     * Compute turret angle (deg) to face a field point (targetX,targetY) given robot pose.
     */
    public double computeAngleToFaceTargetDeg(double targetX, double targetY, Pose robotPose) {
        if (robotPose == null) return getCurrentAngleDeg();

        double rx = robotPose.getX();
        double ry = robotPose.getY();

        double dx = targetX - rx;
        double dy = targetY - ry;

        if (Math.hypot(dx, dy) < FACE_TARGET_MIN_DIST_IN) return getCurrentAngleDeg();

        double targetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // turret 0 = robot forward
        double desiredTurretDeg = -wrapDeg180(targetFieldDeg - robotHeadingDeg - TURRET_MOUNT_OFFSET_DEG);

        return chooseWrappedWithinLimits(desiredTurretDeg, getCurrentAngleDeg());
    }

    public void faceTarget(double targetX, double targetY, Pose robotPose) {
        goToAngle(computeAngleToFaceTargetDeg(targetX, targetY, robotPose));
    }

    public void aimStepDeg(double errorDeg, double deadbandDeg, double maxStepDeg) {
        if (Double.isNaN(errorDeg)) return;

        if (Math.abs(errorDeg) < deadbandDeg) {
            goToAngle(getCurrentAngleDeg()); // hold
            return;
        }

        double step = Range.clip(errorDeg, -maxStepDeg, maxStepDeg);
        goToAngle(getCurrentAngleDeg() + step);
    }

    // =========================
    // Update loop (persist)
    // =========================
    public void update() {
        OpModeStorage.turretAngleOffsetDeg = angleOffsetDeg;
        OpModeStorage.turretTrackEnabled = trackEnabled;

        // Optional: if you want to stop heating the motor when it arrives, uncomment.
        // This does NOT change your goToAngle logic, it just stops after it reports not busy.
        /*
        if (turretMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !turretMotor.isBusy()) {
            turretMotor.setPower(0.0);
        }
        */
    }
}
