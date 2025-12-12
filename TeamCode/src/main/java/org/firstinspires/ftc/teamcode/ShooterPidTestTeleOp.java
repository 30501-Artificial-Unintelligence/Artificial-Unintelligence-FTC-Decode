package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;

@TeleOp(name = "Shooter + Loader TeleOp", group = "Comp")
public class ShooterPidTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;
    private LoaderSubsystem loader;

    // Shooter state
    private boolean shooterEnabled = false; // toggled with X
    // 0 = near, 1 = far (toggled with Y)
    private int fieldPosInput = 0;

    // Edge-detection booleans
    private boolean prevShooterToggle = false;
    private boolean prevFieldToggle = false;
    private boolean prevLoaderButton = false;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new ShooterSubsystem(hardwareMap);
        loader  = new LoaderSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // === BUTTON READS ===
            boolean shooterToggleButton = gamepad1.x;            // toggle shooter on/off
            boolean fieldToggleButton   = gamepad1.y;            // toggle near/far
            boolean loaderButton        = gamepad1.right_bumper; // fire loader once

            boolean rpmUpButton   = gamepad1.dpad_up;
            boolean rpmDownButton = gamepad1.dpad_down;

            // === SHOOTER TOGGLE (X) ===
            if (shooterToggleButton && !prevShooterToggle) {
                shooterEnabled = !shooterEnabled;  // flip state
            }
            prevShooterToggle = shooterToggleButton;

            // === NEAR/FAR TOGGLE (Y) ===
            if (fieldToggleButton && !prevFieldToggle) {
                fieldPosInput = (fieldPosInput == 0) ? 1 : 0;    // 0 -> 1, 1 -> 0
            }
            prevFieldToggle = fieldToggleButton;

            // === LOADER CONTROL (RB) ===
            // Start a single up-then-down cycle on rising edge
            if (loaderButton && !prevLoaderButton) {
                loader.startCycle();
            }
            prevLoaderButton = loaderButton;

            // Must be called every loop so the servo returns after UP_TIME_SEC
            loader.updateLoader();

            // === SHOOTER UPDATE ===
            shooter.update(
                    shooterEnabled,   // shooterOnCommand (toggle)
                    rpmUpButton,      // bump RPM up
                    rpmDownButton,    // bump RPM down
                    fieldPosInput     // 0 = near, 1 = far
            );

            // === TELEMETRY ===
            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Field Mode", (fieldPosInput == 0) ? "NEAR" : "FAR");
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRpmEstimate());
            telemetry.addData("Loader Cycling", loader.isCycling());
            telemetry.update();
        }
    }
}
