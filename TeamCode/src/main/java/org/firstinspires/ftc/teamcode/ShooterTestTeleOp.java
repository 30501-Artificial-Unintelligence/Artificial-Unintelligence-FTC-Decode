package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private SpindexerSubsystem spindexer;
    private LoaderSubsystem loader;

    private boolean prevB = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {
        shooter   = new ShooterSubsystem(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);

        telemetry.addLine("Spindexer auto-intake running.");
        telemetry.addLine("Y: start eject sequence (if any balls).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Home spindexer so slot 0 is at intake (abs ≈ 260.7°)
        spindexer.homeToIntake();

        while (opModeIsActive()) {
            // ===== SHOOTER =====
            shooter.update(
                    gamepad1.a,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down
            );

            // ===== LOADER (manual B) =====
            boolean b = gamepad1.b;
            if (b && !prevB) {
                loader.startCycle();
            }
            prevB = b;
            loader.updateLoader();

            // ===== INTAKE =====
            intake.StartIntake();

            // ===== SPINDEXER =====
            boolean yEdge = gamepad1.y && !lastY;
            lastY = gamepad1.y;

            spindexer.update(telemetry, loader, yEdge);

            // ===== TELEMETRY =====
            SpindexerSubsystem.Ball[] s = spindexer.getSlots();

            boolean readyForIntake = !spindexer.isEjecting() && !spindexer.isAutoRotating();
            telemetry.addData("Spd intakeSlot", spindexer.getIntakeSlotIndex());
            telemetry.addData("Spd readyForIntake", readyForIntake);
            telemetry.addData("Spd full", spindexer.isFull());
            telemetry.addData("Spd ejecting", spindexer.isEjecting());
            telemetry.addData("Spd slot[0]", s[0]);
            telemetry.addData("Spd slot[1]", s[1]);
            telemetry.addData("Spd slot[2]", s[2]);
            telemetry.addData("Spd angle(enc)", "%.1f", spindexer.getCurrentAngleDeg());
            spindexer.debugAbsAngle(telemetry);

            telemetry.addData("Shooter On", shooter.isOn());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Current RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());

            telemetry.update();
        }
    }
}
