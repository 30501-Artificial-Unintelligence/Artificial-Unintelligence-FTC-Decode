package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;

@TeleOp(name = "Shooter PID Test", group = "Comp")
public class ShooterPidTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;
    private LoaderSubsystem loader;

    private FtcDashboard dashboard;

    // Shooter state
    private boolean shooterEnabled = false; // toggled with X
    // 0 = near, 1 = far (toggled with Y)
    private int fieldPosInput = 0;

    // Edge-detection booleans
    private boolean prevShooterToggle = false;
    private boolean prevFieldToggle = false;
    private boolean prevLoaderButton = false;

    // gamepad2 edge-detectors for PID tuning
    private boolean prevG2DpadUp = false;
    private boolean prevG2DpadDown = false;
    private boolean prevG2RB = false;
    private boolean prevG2LB = false;

    // Step sizes for PID tuning
    private static final double KP_STEP = 0.0001;
    private static final double KD_STEP = 0.00005;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new ShooterSubsystem(hardwareMap);
        loader  = new LoaderSubsystem(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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
                shooterEnabled = !shooterEnabled;
            }
            prevShooterToggle = shooterToggleButton;

            // === NEAR/FAR TOGGLE (Y) ===
            if (fieldToggleButton && !prevFieldToggle) {
                fieldPosInput = (fieldPosInput == 0) ? 1 : 0;
            }
            prevFieldToggle = fieldToggleButton;

            // === LOADER CONTROL (RB) ===
            if (loaderButton && !prevLoaderButton) {
                loader.startCycle();
            }
            prevLoaderButton = loaderButton;

            loader.updateLoader();

            // === SHOOTER UPDATE (calls PID inside) ===
            shooter.update(
                    shooterEnabled,
                    rpmUpButton,
                    rpmDownButton,
                    fieldPosInput
            );

            // === PID TUNING (GAMEPAD2) ===
            // kP with dpad up/down
            boolean g2Up = gamepad2.dpad_up;
            boolean g2Down = gamepad2.dpad_down;

            if (g2Up && !prevG2DpadUp) {
                shooter.adjustKp(KP_STEP);
            }
            if (g2Down && !prevG2DpadDown) {
                shooter.adjustKp(-KP_STEP);
            }
            prevG2DpadUp = g2Up;
            prevG2DpadDown = g2Down;

            // kD with bumpers
            boolean g2RB = gamepad2.right_bumper;
            boolean g2LB = gamepad2.left_bumper;

            if (g2RB && !prevG2RB) {
                shooter.adjustKd(KD_STEP);
            }
            if (g2LB && !prevG2LB) {
                shooter.adjustKd(-KD_STEP);
            }
            prevG2RB = g2RB;
            prevG2LB = g2LB;

            // === TELEMETRY ===
            double target = shooter.getTargetRpm();
            double current = shooter.getCurrentRpmEstimate();
            double error = target - current;

            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Field Mode", (fieldPosInput == 0) ? "NEAR" : "FAR");
            telemetry.addData("Target RPM", "%.0f", target);
            telemetry.addData("Current RPM", "%.0f", current);
            telemetry.addData("Error RPM", "%.0f", error);

            telemetry.addData("kP", shooter.getKp());
            telemetry.addData("kI", shooter.getKi());
            telemetry.addData("kD", shooter.getKd());

            telemetry.addData("Loader Cycling", loader.isCycling());
            telemetry.update();

            // === DASHBOARD PACKET (for graphs) ===
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("rpm", current);
            packet.put("target", target);
            packet.put("error", error);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
