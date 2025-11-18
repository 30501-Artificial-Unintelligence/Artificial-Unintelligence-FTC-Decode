package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends LinearOpMode {

    private ShooterSubsystem shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new ShooterSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            boolean buttonPressed = gamepad1.a;  // A toggles shooter
            double gamepad1LeftY = gamepad1.left_stick_y;
            //shooter.update(buttonPressed);
            shooter.joystick(buttonPressed, gamepad1LeftY);

            telemetry.addData("Shooter on?", shooter.isOn());
            telemetry.update();
        }
    }
}
