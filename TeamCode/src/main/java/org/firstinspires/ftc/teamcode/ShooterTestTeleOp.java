package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private boolean prevB = false;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);
        LoaderSubsystem loader = new LoaderSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            // A = toggle on/off
            // dpad_up = +250 RPM
            // dpad_down = -250 RPM
            shooter.update(
                    gamepad1.a,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down
            );
            boolean b = gamepad1.b;

            // On rising edge of A (just pressed)
            if (b && !prevB) {
                loader.startCycle();
            }
            prevB = b;

            // Let the subsystem handle timing and auto-return
            loader.updateLoader();

            intake.StartIntake();

            telemetry.addData("Shooter On", shooter.isOn());
            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Current RPM (est)", "%.0f", shooter.getCurrentRpmEstimate());
            telemetry.update();
        }
    }
}
