package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;

@TeleOp(name = "Shooter Only TeleOp", group = "Test")
public class ShooterOnlyTestTeleOp extends LinearOpMode {

    private ShooterSubsystemFF shooter;

    // 0 = near field, 1 = far field
    private int fieldPos = 0;

    // Rising-edge trackers
    private boolean prevX = false;
    private boolean prevA = false;

    // Latched shooter toggle state
    private boolean shooterOn = false;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystemFF(hardwareMap);

        telemetry.addLine("ShooterOnlyTeleOp ready.");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = toggle shooter on/off");
        telemetry.addLine("  D-pad Up/Down = +250 / -250 RPM (for current field)");
        telemetry.addLine("  X = toggle field position (near/far) + hood angle");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // --- Toggle shooter on/off with A (rising edge) ---
            boolean a = gamepad1.a;
            if (a && !prevA) {
                shooterOn = !shooterOn;
            }
            prevA = a;

            // --- Toggle field position with X (rising edge) ---
            boolean x = gamepad1.x;
            if (x && !prevX) {
                fieldPos = (fieldPos == 0) ? 1 : 0;
            }
            prevX = x;

            // --- RPM adjust buttons (held; subsystem edge-detects internally) ---
            boolean increasePressed = gamepad1.dpad_up;    // +RPM for current field
            boolean decreasePressed = gamepad1.dpad_down;  // -RPM for current field

            // --- Update shooter subsystem ---
            shooter.update(
                    shooterOn,
                    increasePressed,
                    decreasePressed,
                    fieldPos     // 0 = near, 1 = far
            );

            // --- Telemetry ---
            telemetry.addData("Shooter On (latched)", shooterOn);
            telemetry.addData("Shooter On (subsystem)", shooter.isOn());
            telemetry.addData("Field Position", fieldPos == 0 ? "Near (0)" : "Far (1)");
            telemetry.addData("Near RPM", "%.0f", shooter.getNearRpm());
            telemetry.addData("Far RPM", "%.0f", shooter.getFarRpm());
            telemetry.addData("Current Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("RPM (estimate)", "%.0f", shooter.getCurrentRpmEstimate());

            // Optional debug if you want it:
            telemetry.addData("Target TPS", "%.0f", shooter.getTargetTps());
            telemetry.addData("Vel TPS", "%.0f", shooter.getVelocityTps());
            telemetry.addData("Power Cmd", "%.3f", shooter.getPowerCmd());
            telemetry.addData("FF", "%.3f", shooter.getFF());

            telemetry.update();

            // Yield time back to the system
            idle();
        }

        // Make sure shooter is off when OpMode ends
        shooter.stop();
    }
}
