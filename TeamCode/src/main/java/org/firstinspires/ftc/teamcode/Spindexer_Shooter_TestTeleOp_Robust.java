package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State_new_Incremental;

@TeleOp(name = "Spindexer+Shooter Test (Robust)", group = "Test")
public class Spindexer_Shooter_TestTeleOp_Robust extends OpMode {

    private SpindexerSubsystem_State_new_Incremental spindexer;
    private ShooterSubsystemFF shooter;

    // ===== Spindexer controls (gamepad1) =====
    private static final double STICK_DEADBAND = 0.08;
    public static double MANUAL_DEG_PER_SEC = 360.0;

    private boolean autoSpinEnabled = false;
    private int autoDirSign = +1; // +1 or -1
    public static double AUTO_DEG_PER_SEC = 90.0;
    public static double AUTO_SPEED_STEP = 15.0;
    public static double AUTO_SPEED_MIN = 0.0;
    public static double AUTO_SPEED_MAX = 720.0;

    private boolean stepInProgress = false;
    private double stepTargetDeg = 0.0;
    public static double STEP_DEG = 120.0;
    public static double STEP_DONE_TOL_DEG = 2.5;

    // ===== Spin 360 "full power" (gamepad1 LB) =====
    // Matches your spindexer motor encoder constant:
    private static final double SPIN_TICKS_PER_REV = 384.5;
    private static final double DEG_PER_TICK = 360.0 / SPIN_TICKS_PER_REV;

    public static double SPIN360_TARGET_DEG = 720.0;
    public static double SPIN360_AHEAD_DEG = 179.0;          // keep big error -> saturate power
    public static long   SPIN360_TIMEOUT_MS = 3500;          // safety timeout
    public static double SPIN360_BOOST_MAX_POWER = 1.0;      // temporarily override MAX_POWER/EJECT

    private boolean spin360Active = false;
    private double spin360MovedDeg = 0.0;
    private int spin360LastTicks = 0;
    private long spin360StartMs = 0;

    private double savedMaxPower = 0.0;
    private double savedMaxPowerEject = 0.0;
    private boolean spin360Boosted = false;

    // ===== Shooter controls (either gamepad) =====
    private boolean shooterOn = false;
    private boolean prevShooterToggle = false;

    // Prime pulses so target RPM ramps up after enabling (if your shooter starts at 0 RPM)
    public static int PRIME_UP_EDGES_ON_ENABLE = 16;
    private int primeEdgesRemaining = 0;
    private boolean primePulseHigh = false;

    // Keep 0 for test unless you want far/near logic
    private int shooterFieldPos = 0;

    // timing
    private long lastLoopNs = 0;

    // edges (spindexer)
    private boolean prevA = false, prevB = false, prevX = false;
    private boolean prevDpadUp = false, prevDpadDown = false;
    private boolean prevLB = false;

    @Override
    public void init() {
        spindexer = new SpindexerSubsystem_State_new_Incremental(hardwareMap);
        shooter   = new ShooterSubsystemFF(hardwareMap);

        // Freeze spindexer auto-intake logic by making it "full" during this test.
        spindexer.presetSlots(
                SpindexerSubsystem_State_new_Incremental.Ball.PURPLE,
                SpindexerSubsystem_State_new_Incremental.Ball.PURPLE,
                SpindexerSubsystem_State_new_Incremental.Ball.PURPLE
        );

        lastLoopNs = System.nanoTime();

        telemetry.addLine("Spindexer (GP1): LSX manual | A auto | X dir | B step120 | DpadUp/Down autoSpeed | LB spin360 full");
        telemetry.addLine("Shooter: (GP1 or GP2) Y or RB toggle | GP2 DpadUp/Down speed | Triggers also work");
        telemetry.update();
    }

    @Override
    public void start() {
        spindexer.setTargetAngleDeg(spindexer.getCurrentAngleDeg());

        shooterOn = false;
        primeEdgesRemaining = 0;
        primePulseHigh = false;

        spin360Active = false;
        spin360MovedDeg = 0.0;
        spin360Boosted = false;

        lastLoopNs = System.nanoTime();
    }

    private void restoreSpin360BoostIfNeeded() {
        if (spin360Boosted) {
            SpindexerSubsystem_State_new_Incremental.MAX_POWER = savedMaxPower;
            SpindexerSubsystem_State_new_Incremental.MAX_POWER_EJECT = savedMaxPowerEject;
            spin360Boosted = false;
        }
    }

    @Override
    public void loop() {
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastLoopNs) / 1e9;
        lastLoopNs = nowNs;
        dt = Range.clip(dt, 0.001, 0.05);

        long nowMs = System.currentTimeMillis();

        // =========================
        // ===== SHOOTER INPUTS =====
        // =========================
        boolean shooterToggleBtn =
                gamepad1.y || gamepad2.y || gamepad1.right_bumper || gamepad2.right_bumper;

        boolean shooterToggleEdge = shooterToggleBtn && !prevShooterToggle;
        prevShooterToggle = shooterToggleBtn;

        if (shooterToggleEdge) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                primeEdgesRemaining = PRIME_UP_EDGES_ON_ENABLE;
                primePulseHigh = false;
            } else {
                primeEdgesRemaining = 0;
                primePulseHigh = false;
            }
        }

        // Shooter speed controls: prefer gamepad2 dpad (doesn't conflict with spindexer dpad on gamepad1)
        boolean shooterUpCmd =
                gamepad2.dpad_up || gamepad2.right_trigger > 0.6 || gamepad1.right_trigger > 0.6;

        boolean shooterDownCmd =
                gamepad2.dpad_down || gamepad2.left_trigger > 0.6 || gamepad1.left_trigger > 0.6;

        // Prime: generate short "up" edges for a moment after enabling
        boolean primeUpThisLoop = false;
        if (shooterOn && primeEdgesRemaining > 0) {
            if (!primePulseHigh) {
                primeUpThisLoop = true;   // one-loop high
                primePulseHigh = true;
                primeEdgesRemaining--;
            } else {
                primeUpThisLoop = false;  // one-loop gap
                primePulseHigh = false;
            }
        }

        boolean shooterUpEffective = shooterUpCmd || primeUpThisLoop;

        shooter.update(
                shooterOn,
                shooterUpEffective,
                shooterDownCmd,
                shooterFieldPos
        );

        // =========================
        // ===== SPINDEXER INPUTS ===
        // =========================
        double stick = gamepad1.left_stick_x;
        boolean manualActive = Math.abs(stick) > STICK_DEADBAND;

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        boolean lb = gamepad1.left_bumper;

        boolean aEdge = a && !prevA;
        boolean bEdge = b && !prevB;
        boolean xEdge = x && !prevX;
        boolean upEdge = dpadUp && !prevDpadUp;
        boolean downEdge = dpadDown && !prevDpadDown;
        boolean lbEdge = lb && !prevLB;

        prevA = a; prevB = b; prevX = x;
        prevDpadUp = dpadUp; prevDpadDown = dpadDown;
        prevLB = lb;

        if (aEdge) autoSpinEnabled = !autoSpinEnabled;
        if (xEdge) autoDirSign *= -1;

        if (upEdge) AUTO_DEG_PER_SEC = Range.clip(AUTO_DEG_PER_SEC + AUTO_SPEED_STEP, AUTO_SPEED_MIN, AUTO_SPEED_MAX);
        if (downEdge) AUTO_DEG_PER_SEC = Range.clip(AUTO_DEG_PER_SEC - AUTO_SPEED_STEP, AUTO_SPEED_MIN, AUTO_SPEED_MAX);

        // Step +120°
        if (bEdge) {
            double cur = spindexer.getCurrentAngleDeg();
            stepTargetDeg = cur + autoDirSign * STEP_DEG;
            spindexer.setTargetAngleDeg(stepTargetDeg);
            stepInProgress = true;
            spin360Active = false;
            restoreSpin360BoostIfNeeded();
        }

        // Spin 360 "full power"
        if (lbEdge) {
            spin360Active = true;
            spin360MovedDeg = 0.0;
            spin360LastTicks = spindexer.getEncoder();
            spin360StartMs = nowMs;

            // override other modes while spinning 360
            stepInProgress = false;

            // BOOST power limits so "full speed" doesn't get capped
            if (!spin360Boosted) {
                savedMaxPower = SpindexerSubsystem_State_new_Incremental.MAX_POWER;
                savedMaxPowerEject = SpindexerSubsystem_State_new_Incremental.MAX_POWER_EJECT;

                SpindexerSubsystem_State_new_Incremental.MAX_POWER = SPIN360_BOOST_MAX_POWER;
                SpindexerSubsystem_State_new_Incremental.MAX_POWER_EJECT = SPIN360_BOOST_MAX_POWER;

                spin360Boosted = true;
            }
        }

        // Manual cancels step + 360 immediately (instant recovery)
        if (manualActive) {
            stepInProgress = false;
            spin360Active = false;
            restoreSpin360BoostIfNeeded();
        }

        // =========================
        // ===== Command priority ===
        // =========================
        if (spin360Active) {
            // Measure physical progress from encoder ticks
            int curTicks = spindexer.getEncoder();
            int dTicks = curTicks - spin360LastTicks;
            spin360MovedDeg += Math.abs(dTicks) * DEG_PER_TICK;
            spin360LastTicks = curTicks;

            // KEEP TARGET ~179° AHEAD of current -> large error -> saturate motor power
            spindexer.setTargetAngleDeg(spindexer.getCurrentAngleDeg() + autoDirSign * SPIN360_AHEAD_DEG);

            boolean doneByDistance = (spin360MovedDeg >= SPIN360_TARGET_DEG);
            boolean doneByTimeout  = (nowMs - spin360StartMs >= SPIN360_TIMEOUT_MS);

            if (doneByDistance || doneByTimeout) {
                spin360Active = false;

                // Hold exactly where we are NOW
                spindexer.setTargetAngleDeg(spindexer.getCurrentAngleDeg());

                restoreSpin360BoostIfNeeded();
            }

        } else if (stepInProgress) {
            // Hold step target until reached
            if (Math.abs(spindexer.getAngleErrorToTargetDeg()) <= STEP_DONE_TOL_DEG) {
                stepInProgress = false;
            }

        } else if (manualActive) {
            // Manual "spin": walk target angle using joystick
            spindexer.setTargetAngleDeg(spindexer.getTargetAngleDeg() + stick * MANUAL_DEG_PER_SEC * dt);

        } else if (autoSpinEnabled && AUTO_DEG_PER_SEC > 0.0) {
            // Auto spin: walk target angle
            spindexer.setTargetAngleDeg(spindexer.getTargetAngleDeg() + autoDirSign * AUTO_DEG_PER_SEC * dt);
        }

        // Run spindexer PID (yEdge false so eject never triggers; loader null is safe)
        spindexer.update(telemetry, null, false, -1);

        // =========================
        // ===== TELEMETRY ==========
        // =========================
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("PrimeEdgesRemaining", primeEdgesRemaining);
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
        telemetry.addData("RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());

        telemetry.addLine("=== SPINDEXER ===");
        telemetry.addData("manualActive", manualActive);
        telemetry.addData("autoEnabled", autoSpinEnabled);
        telemetry.addData("autoDir", (autoDirSign > 0) ? "+ (CW)" : "- (CCW)");
        telemetry.addData("autoSpeed(deg/s)", "%.1f", AUTO_DEG_PER_SEC);

        telemetry.addData("stepInProgress", stepInProgress);
        if (stepInProgress) telemetry.addData("stepTargetDeg", "%.1f", stepTargetDeg);

        telemetry.addData("spin360Active", spin360Active);
        telemetry.addData("spin360MovedDeg", "%.1f", spin360MovedDeg);
        telemetry.addData("boosted", spin360Boosted);
        telemetry.addData("MAX_POWER", "%.2f", SpindexerSubsystem_State_new_Incremental.MAX_POWER);
        telemetry.addData("MAX_POWER_EJECT", "%.2f", SpindexerSubsystem_State_new_Incremental.MAX_POWER_EJECT);

        telemetry.addData("curAngle", "%.1f", spindexer.getCurrentAngleDeg());
        telemetry.addData("tgtAngle", "%.1f", spindexer.getTargetAngleDeg());
        telemetry.addData("errDeg", "%.2f", spindexer.getAngleErrorToTargetDeg());
        telemetry.addData("encoder", spindexer.getEncoder());
        telemetry.addData("targetTicks", spindexer.getTarget());
        telemetry.addData("dt(ms)", "%.1f", dt * 1000.0);

        telemetry.update();
    }
}
