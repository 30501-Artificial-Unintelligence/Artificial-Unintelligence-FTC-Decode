package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "TurretTestTeleOp", group = "Test")
public class TurretTestTeleOp extends OpMode {

    private TurretSubsystem turret;
    private boolean prevA, prevB, prevX;

    @Override
    public void init() {
        turret = new TurretSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        // --- Presets using RUN_TO_POSITION ---
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;

        if (a && !prevA) turret.goToAngle(0.0);
        if (b && !prevB) turret.goToAngle(45.0);
        if (x && !prevX) turret.goToAngle(-45.0);

        prevA = a;
        prevB = b;
        prevX = x;

        // --- Manual override with joystick ---
        double stickX = gamepad1.right_stick_x;
        if (Math.abs(stickX) > 0.05) {
            turret.setManualPower(stickX * 0.4);  // manual mode, overrides auto
        }

        turret.update();

        telemetry.addData("Turret angle", "%.1f", turret.getCurrentAngleDeg());
        telemetry.addData("Target angle", "%.1f", turret.getTargetAngleDeg());
        telemetry.addData("Mode", turretMotorModeString());
        telemetry.update();
    }

    private String turretMotorModeString() {
        // little helper if you want to print the underlying motor mode
        return ""; // optional; you can look at turretMotor.getMode() inside the subsystem
    }
}
