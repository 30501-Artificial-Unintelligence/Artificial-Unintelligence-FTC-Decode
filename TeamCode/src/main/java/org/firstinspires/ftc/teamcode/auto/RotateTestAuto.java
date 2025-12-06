package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Autonomous(name = "RotateTestAuto", group = "Test")
public class RotateTestAuto extends LinearOpMode {

    private DrivetrainSubsystem drivetrain;

    @Override
    public void runOpMode() {
        drivetrain = new DrivetrainSubsystem(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            // Rotate one way for 1 second
            drivetrain.rotateInPlaceForMs(this, 0.4, 1000);
            sleep(500);

            // Rotate the other way for 1 second
            drivetrain.rotateInPlaceForMs(this, -0.4, 1000);
            sleep(500);

            // Stop at end just in case
            drivetrain.stopAll();
        }
    }
}
