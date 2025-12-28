package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystemPIDF;

@Configurable
@TeleOp(name = "TUNE Spindexer PIDF + ABS (PIDF version)", group = "Tuning")
public class TuneSpindexerPidfAbs extends OpMode {

    // ===== Command target =====
    public static double targetDeg = 0.0;
    public static double fineStepDeg = 5.0;
    public static double slotStepDeg = 120.0;

    // Pattern override (0/21/22/23)
    public static int patternOverride = 0;

    // ===== Tunables that ACTUALLY exist in SpindexerSubsystemPIDF.Tune =====
    public static double kP = 10.0, kI = 0.0, kD = 0.0, kF = 0.0;

    public static double ABS_MECH_OFFSET_DEG = 245.0;
    public static boolean ABS_ENABLE_SANITY = true;
    public static double ABS_REZERO_THRESHOLD_DEG = 3.0;
    public static int    ABS_REZERO_STREAK = 3;
    public static long   ABS_REZERO_COOLDOWN_MS = 600;
    public static double ABS_MAX_JUMP_DEG = 25.0;
    public static double ABS_MIN_VOLT = 0.05;
    public static double ABS_MAX_VOLT = 3.25;
    public static int    ABS_AVG_SAMPLES = 5;

    public static boolean ABS_POSTMOVE_NUDGE = true;
    public static double  ABS_POSTMOVE_TOL_DEG = 2.0;
    public static double  ABS_POSTMOVE_POWER = 0.25;
    public static int     ABS_POSTMOVE_TIMEOUT_MS = 350;

    public static double MOVE_POWER = 0.55;
    public static int    MOVE_TIMEOUT_MS = 700;

    private SpindexerSubsystemPIDF spindexer;
    private LoaderSubsystem loader;

    private boolean lastA, lastB, lastX, lastY;
    private boolean lastUp, lastDown, lastLeft, lastRight;
    private boolean lastLB, lastRB;

    @Override
    public void init() {
        spindexer = new SpindexerSubsystemPIDF(hardwareMap);
        loader = new LoaderSubsystem(hardwareMap);

        telemetry.addLine("Spindexer tuner ready.");
        telemetry.addLine("Dpad L/R: ±120deg, Dpad U/D: ±fineStep");
        telemetry.addLine("A: go to target | B: home(0) | Y: start eject");
        telemetry.addLine("X: (optional) force rezero - add method in subsystem if you want it");
        telemetry.update();
    }

    @Override
    public void loop() {
        pushTunablesIntoSubsystem();

        boolean aEdge = gamepad1.a && !lastA;
        boolean bEdge = gamepad1.b && !lastB;
        boolean xEdge = gamepad1.x && !lastX;
        boolean yEdge = gamepad1.y && !lastY;

        boolean upEdge = gamepad1.dpad_up && !lastUp;
        boolean downEdge = gamepad1.dpad_down && !lastDown;
        boolean leftEdge = gamepad1.dpad_left && !lastLeft;
        boolean rightEdge = gamepad1.dpad_right && !lastRight;

        boolean lbEdge = gamepad1.left_bumper && !lastLB;
        boolean rbEdge = gamepad1.right_bumper && !lastRB;

        if (leftEdge)  targetDeg -= slotStepDeg;
        if (rightEdge) targetDeg += slotStepDeg;
        if (upEdge)    targetDeg += fineStepDeg;
        if (downEdge)  targetDeg -= fineStepDeg;

        if (bEdge) targetDeg = 0.0;

        // If you add a public method in the subsystem, you can enable this:
        // if (xEdge) spindexer.rezeroFromAbsNow();

        if (aEdge) spindexer.moveToAngleAsync(targetDeg, MOVE_POWER);

        if (lbEdge) { targetDeg -= slotStepDeg; spindexer.moveToAngleAsync(targetDeg, MOVE_POWER); }
        if (rbEdge) { targetDeg += slotStepDeg; spindexer.moveToAngleAsync(targetDeg, MOVE_POWER); }

        spindexer.update(telemetry, loader, yEdge, patternOverride);
        loader.updateLoader();

        double encDeg = spindexer.getCurrentAngleDeg();
        Double absDeg = spindexer.getAbsInternalAngleDeg();
        Double diffDeg = spindexer.getEncMinusAbsDeg();

        telemetry.addLine("=== Spindexer PIDF + ABS Tuning ===");
        telemetry.addData("TargetDeg", "%.1f", targetDeg);
        telemetry.addData("EncDeg", "%.1f", encDeg);
        telemetry.addData("AbsDeg", absDeg == null ? "null" : String.format("%.1f", absDeg));
        telemetry.addData("Err(enc-abs)", diffDeg == null ? "null" : String.format("%.2f deg", diffDeg));

        telemetry.addData("EncTicks", spindexer.getEncoderTicks());
        telemetry.addData("TargetTicks", spindexer.getTargetTicks());
        telemetry.addData("Busy", spindexer.isAutoRotating() || spindexer.isEjecting());

        telemetry.update();

        lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x; lastY = gamepad1.y;
        lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
        lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;
    }

    private void pushTunablesIntoSubsystem() {
        // PIDF
        SpindexerSubsystemPIDF.Tune.kP = kP;
        SpindexerSubsystemPIDF.Tune.kI = kI;
        SpindexerSubsystemPIDF.Tune.kD = kD;
        SpindexerSubsystemPIDF.Tune.kF = kF;

        // Motion
        SpindexerSubsystemPIDF.Tune.MOVE_POWER = MOVE_POWER;
        SpindexerSubsystemPIDF.Tune.MOVE_TIMEOUT_MS = MOVE_TIMEOUT_MS;

        // ABS
        SpindexerSubsystemPIDF.Tune.ABS_MECH_OFFSET_DEG = ABS_MECH_OFFSET_DEG;
        SpindexerSubsystemPIDF.Tune.ABS_ENABLE_SANITY = ABS_ENABLE_SANITY;
        SpindexerSubsystemPIDF.Tune.ABS_REZERO_THRESHOLD_DEG = ABS_REZERO_THRESHOLD_DEG;
        SpindexerSubsystemPIDF.Tune.ABS_REZERO_STREAK = ABS_REZERO_STREAK;
        SpindexerSubsystemPIDF.Tune.ABS_REZERO_COOLDOWN_MS = ABS_REZERO_COOLDOWN_MS;
        SpindexerSubsystemPIDF.Tune.ABS_MAX_JUMP_DEG = ABS_MAX_JUMP_DEG;
        SpindexerSubsystemPIDF.Tune.ABS_MIN_VOLT = ABS_MIN_VOLT;
        SpindexerSubsystemPIDF.Tune.ABS_MAX_VOLT = ABS_MAX_VOLT;
        SpindexerSubsystemPIDF.Tune.ABS_AVG_SAMPLES = ABS_AVG_SAMPLES;

        // Post-move nudge
        SpindexerSubsystemPIDF.Tune.ABS_POSTMOVE_NUDGE = ABS_POSTMOVE_NUDGE;
        SpindexerSubsystemPIDF.Tune.ABS_POSTMOVE_TOL_DEG = ABS_POSTMOVE_TOL_DEG;
        SpindexerSubsystemPIDF.Tune.ABS_POSTMOVE_POWER = ABS_POSTMOVE_POWER;
        SpindexerSubsystemPIDF.Tune.ABS_POSTMOVE_TIMEOUT_MS = ABS_POSTMOVE_TIMEOUT_MS;
    }
}
