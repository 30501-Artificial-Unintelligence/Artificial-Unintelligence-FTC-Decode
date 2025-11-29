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

    // spindexer vars
    private boolean lastY = false;

    // Eject sequence state
    private boolean ejecting = false;
    private int ejectSlotIndex = 0;
    private long ejectPhaseTime = 0;
    private int ejectPhase = 0; // 0 = find/rotate, 1 = wait before loader, 2 = wait after loader

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);
        loader  = new LoaderSubsystem(hardwareMap);
        intake  = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);

        telemetry.addLine("Spindexer auto-intake running.");
        telemetry.addLine("Y: start eject sequence (if any balls).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

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

            // ===== SPINDEXER AUTO-INTAKE =====
            spindexer.update(telemetry);

            // ===== START EJECT SEQUENCE WITH Y =====
            boolean yEdge = gamepad1.y && !lastY;  lastY = gamepad1.y;
            if (yEdge && !ejecting && spindexer.hasAnyBall()) {
                ejecting = true;
                ejectSlotIndex = 0;
                ejectPhase = 0;
            }

            // ===== EJECT STATE MACHINE =====
            if (ejecting) {
                long now = System.currentTimeMillis();

                if (ejectPhase == 0) {
                    // Find next non-empty slot
                    if (ejectSlotIndex >= 3 || !spindexer.hasAnyBall()) {
                        // Done ejecting: go back so slot 0 is at the intake position
                        spindexer.homeToIntake();
                        ejecting = false;
                    } else if (!spindexer.slotHasBall(ejectSlotIndex)) {
                        // Skip empty slot
                        ejectSlotIndex++;
                    } else {
                        // Rotate this slot to LOAD (blocking move with timeout)
                        spindexer.moveSlotToLoadBlocking(ejectSlotIndex);
                        ejectPhaseTime = now + 100; // wait 0.1s before loader fires
                        ejectPhase = 1;
                    }
                } else if (ejectPhase == 1) {
                    // After 0.1s at LOAD, fire the loader
                    if (now >= ejectPhaseTime) {
                        loader.startCycle();
                        ejectPhaseTime = now + 100; // wait another 0.1s after firing
                        ejectPhase = 2;
                    }
                } else if (ejectPhase == 2) {
                    // After extra 0.1s, mark slot empty and move to next
                    if (now >= ejectPhaseTime) {
                        spindexer.clearSlot(ejectSlotIndex);
                        ejectSlotIndex++;
                        ejectPhase = 0;
                    }
                }
            }

            // ===== TELEMETRY =====
            SpindexerSubsystem.Ball[] s = spindexer.getSlots();

            boolean readyForIntake = !ejecting && !spindexer.isAutoRotating();
            telemetry.addData("Spd intakeSlot", spindexer.getIntakeSlotIndex());
            telemetry.addData("Spd readyForIntake", readyForIntake);
            telemetry.addData("Spd full", spindexer.isFull());
            telemetry.addData("Spd ejecting", ejecting);
            telemetry.addData("Spd slot[0]", s[0]);
            telemetry.addData("Spd slot[1]", s[1]);
            telemetry.addData("Spd slot[2]", s[2]);
            telemetry.addData("Spd angle(enc)", "%.1f", spindexer.getCurrentAngleDeg()); // <--- NEW
            spindexer.debugAbsAngle(telemetry);

            telemetry.addData("Shooter On", shooter.isOn());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Current RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());

            telemetry.update();

        }
    }
}
