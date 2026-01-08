package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_State_new_Incremental;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerTuningConfig_new;

@TeleOp(name="TUNE Spindexer PIDF (Panels)_Incremental", group="Tuning")
public class SpindexerPIDFTunerTeleOp_Incremental extends OpMode {

    private SpindexerSubsystem_State_new_Incremental spindexer;
    private IntakeSubsystem_Motor intake;

    private Telemetry panelsTelemetry;

    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;

    @Override
    public void init() {
        spindexer = new SpindexerSubsystem_State_new_Incremental(hardwareMap);
        intake = new IntakeSubsystem_Motor(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();

        telemetry.addLine("Spindexer PIDF Tuner (Incremental) Ready");
        telemetry.addLine("Y: trigger eject sequence (spindexer only)");
        telemetry.addLine("A: home slot0 @ INTAKE here");
        telemetry.addLine("B: home slot0 @ LOAD here");
        telemetry.addLine("X: homeToIntake()");
        telemetry.addLine("Tune kP/kI/kD/kF in Panels: SpindexerTuningConfig_new");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean yEdge = gamepad1.y && !lastY;
        lastY = gamepad1.y;

        boolean aEdge = gamepad1.a && !lastA;
        lastA = gamepad1.a;

        boolean bEdge = gamepad1.b && !lastB;
        lastB = gamepad1.b;

        boolean xEdge = gamepad1.x && !lastX;
        lastX = gamepad1.x;

        // Quick homing helpers (super useful during tuning)
        if (aEdge) spindexer.homeSlot0AtIntakeHere();
        if (bEdge) spindexer.homeSlot0AtLoadHere();
        if (xEdge) spindexer.homeToIntake();

        // Tag override can still come from Panels config
        int tagOverride = SpindexerTuningConfig_new.patternTagOverride;

        // Run spindexer state machine + PIDF.
        // Pass loader = null so only the spindexer spins.
        spindexer.update(panelsTelemetry, null, yEdge, tagOverride);

        // Keep intake running so you can test real acquisition timing
        intake.startIntake();

        // ====== Driver Station telemetry (quick view) ======
        double curr = spindexer.getCurrentAngleDeg();
        double tgt  = spindexer.getTargetAngleDeg();

        telemetry.addData("AngleCurr", "%.1f", curr);
        telemetry.addData("AngleTarget", "%.1f", tgt);
        telemetry.addData("AngleErr", "%.1f", (tgt - curr));

        telemetry.addData("Enc", spindexer.getEncoder());
        telemetry.addData("TargetTicks", spindexer.getTarget());

        telemetry.addData("IntakeIndex", spindexer.getIntakeIndex());
        telemetry.addData("HasAnyBall", spindexer.hasAnyBall());
        telemetry.addData("IsFull", spindexer.isFull());
        telemetry.addData("IsEjecting", spindexer.isEjecting());
        telemetry.addData("Pattern", spindexer.getGamePattern());

        telemetry.addData("kP", SpindexerTuningConfig_new.kP);
        telemetry.addData("kI", SpindexerTuningConfig_new.kI);
        telemetry.addData("kD", SpindexerTuningConfig_new.kD);
        telemetry.addData("kF", SpindexerTuningConfig_new.kF);

        telemetry.update();

        // ====== PANELS telemetry (for graphs) ======
        panelsTelemetry.addData("sp_angle_curr", "%.2f", curr);
        panelsTelemetry.addData("sp_angle_tgt",  "%.2f", tgt);
        panelsTelemetry.addData("sp_angle_err",  "%.2f", (tgt - curr));

        panelsTelemetry.addData("sp_enc", spindexer.getEncoder());
        panelsTelemetry.addData("sp_tgt_ticks", spindexer.getTarget());

        panelsTelemetry.addData("sp_intake_idx", spindexer.getIntakeIndex());
        panelsTelemetry.addData("sp_has_ball", spindexer.hasAnyBall() ? 1 : 0);
        panelsTelemetry.addData("sp_is_full",  spindexer.isFull() ? 1 : 0);
        panelsTelemetry.addData("sp_is_ejecting", spindexer.isEjecting() ? 1 : 0);

        panelsTelemetry.addData("sp_kP", SpindexerTuningConfig_new.kP);
        panelsTelemetry.addData("sp_kI", SpindexerTuningConfig_new.kI);
        panelsTelemetry.addData("sp_kD", SpindexerTuningConfig_new.kD);
        panelsTelemetry.addData("sp_kF", SpindexerTuningConfig_new.kF);

        panelsTelemetry.update();
    }
}
