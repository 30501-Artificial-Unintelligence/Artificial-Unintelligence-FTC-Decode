package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Autonomous(name = "Auto_Shoot3_ThenCycle", group = "Comp")
public class Auto_Shoot3_ThenCycle extends LinearOpMode {

    private ShooterSubsystem shooter;
    private IntakeSubsystem_Motor intake;
    private SpindexerSubsystem spindexer;
    private LoaderSubsystem loader;
    private DrivetrainSubsystem drive;  // you will plug your moves here

    // 0 = nearfield, 1 = far field (change this if you want near)
    private static final int AUTO_FIELD_POS = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter   = new ShooterSubsystem(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        drive     = new DrivetrainSubsystem(hardwareMap);

        // Make sure spindexer is homed at start
        spindexer.homeToIntake();

        telemetry.addLine("Auto ready: shooter + spindexer + intake");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ============================================
        // PHASE 1: MOVE TO FIRST SHOOTING POSITION
        // ============================================
        // TODO: Replace this with your actual drive path to first shooting spot
        // Example:
        // drive.driveForwardDistance(24.0);
        // sleep(250);
        idleForMs(250);  // placeholder so loop doesn't exit instantly

        // ============================================
        // PHASE 2: SHOOT FIRST 3 PRELOADED BALLS (FASTEST ORDER)
        // ============================================
        shootAllFastestOnce(AUTO_FIELD_POS);

        // ============================================
        // PHASE 3: MOVE + INTAKE WHILE MOVING
        // ============================================
        // You said you'll handle the movement; this just runs intake+spindexer like TeleOp.
        // Replace the idleForMs(...) with your drive code inside this loop if you want.
        intakeAndAutoIndexForMs(3000);  // 3 seconds of intake while driving (tweak as needed)

        // ============================================
        // PHASE 4: MOVE TO SECOND SHOOTING POSITION
        // ============================================
        // TODO: Replace with your second drive segment to final shooting spot
        // Example:
        // drive.strafeRightDistance(18.0);
        // sleep(250);
        idleForMs(250);  // placeholder

        // ============================================
        // PHASE 5: SHOOT WHATEVER WE HAVE (0–3 BALLS), FASTEST ORDER
        // ============================================
        shootAllFastestOnce(AUTO_FIELD_POS);

        // Clean up
        intake.stopIntake();
        shooter.stop();
    }

    // ----------------------------------------------------------------
    // Helper: simple idle loop so subsystems can run without driving
    // ----------------------------------------------------------------
    private void idleForMs(long ms) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < ms) {
            // Keep shooter running (always true so it toggles ON once and stays on)
            shooter.update(true, false, false, AUTO_FIELD_POS);

            // No eject, no pattern (fast mode)
            spindexer.update(telemetry, loader, false, 0);
            loader.updateLoader();

            telemetry.addData("Phase", "Idle");
            telemetry.update();
            sleep(10);
        }
    }

    // ----------------------------------------------------------------
    // Helper: Shoot all currently loaded balls in fastest order (patternTag=0)
    //  - Spins shooter (using existing ShooterSubsystem)
    //  - Starts eject sequence once
    //  - Lets your existing spindexer state machine run until done
    // ----------------------------------------------------------------
    private void shootAllFastestOnce(int fieldPos) {
        // Optional: small pre-spin-up before we start ejecting
        long spinUpStart = System.currentTimeMillis();
        long spinUpTimeMs = 500;  // tweak if you want more spin-up time

        while (opModeIsActive() && System.currentTimeMillis() - spinUpStart < spinUpTimeMs) {
            shooter.update(true, false, false, fieldPos);             // shooter ON, correct fieldPos
            spindexer.update(telemetry, loader, false, 0);            // no eject yet, fast pattern
            loader.updateLoader();

            telemetry.addData("Phase", "Pre-spin shooter");
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRpmEstimate());
            telemetry.update();
            sleep(10);
        }

        // === Start eject sequence exactly once (yEdge = true once, then false) ===
        boolean yEdge = true;
        spindexer.update(telemetry, loader, yEdge, 0);   // patternTagOverride = 0 → fastest
        loader.updateLoader();
        yEdge = false;

        // === Keep looping until spindexer is done ejecting ===
        while (opModeIsActive() && (spindexer.isEjecting() || spindexer.hasAnyBall())) {
            shooter.update(true, false, false, fieldPos);          // keep shooter running
            spindexer.update(telemetry, loader, false, 0);        // continue eject state machine
            loader.updateLoader();                                // actually move loader servo

            SpindexerSubsystem.Ball[] slots = spindexer.getSlots();
            telemetry.addData("Phase", "Shooting (fast order)");
            telemetry.addData("Slot[0]", slots[0]);
            telemetry.addData("Slot[1]", slots[1]);
            telemetry.addData("Slot[2]", slots[2]);
            telemetry.addData("Ejecting", spindexer.isEjecting());
            telemetry.update();

            sleep(10);
        }

        // When finished, home the spindexer back to intake
        spindexer.homeToIntake();
    }

    // ----------------------------------------------------------------
    // Helper: Intake + auto-index like TeleOp for a fixed amount of time
    //  - Intake motor runs forward
    //  - Spindexer auto-intakes using your existing update()
    //  - No eject (yEdge = false), fast pattern
    //  - You can drop drive code into this loop later
    // ----------------------------------------------------------------
    private void intakeAndAutoIndexForMs(long ms) {
        long start = System.currentTimeMillis();
        intake.startIntake();

        while (opModeIsActive() && (System.currentTimeMillis() - start) < ms) {
            // TODO: Plug in drivetrain movement here if you want it moving while intaking
            // Example:
            // drive.drive(someX, someY, someTurn);

            // Shooter can stay spinning (saves time before the next volley)
            shooter.update(true, false, false, AUTO_FIELD_POS);

            // No eject (yEdge=false), patternTag=0 => "fastest possible"
            spindexer.update(telemetry, loader, false, 0);
            loader.updateLoader();

            SpindexerSubsystem.Ball[] slots = spindexer.getSlots();
            telemetry.addData("Phase", "Intaking + Auto-index");
            telemetry.addData("Slot[0]", slots[0]);
            telemetry.addData("Slot[1]", slots[1]);
            telemetry.addData("Slot[2]", slots[2]);
            telemetry.addData("Spd full", spindexer.isFull());
            telemetry.update();

            sleep(10);
        }

        intake.stopIntake();
    }
}
