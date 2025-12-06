package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Autonomous(name = "DriveForwardMmTest", group = "Test")
public class DriveForwardMmTestAuto extends LinearOpMode {

    private DrivetrainSubsystem drivetrain;

    @Override
    public void runOpMode() {
        drivetrain = new DrivetrainSubsystem(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward 600 mm (0.6 m) at 50% power
            drivetrain.driveForwardMm(this, 600.0, 0.5);

            sleep(500);

            // Drive back 600 mm
            drivetrain.driveForwardMm(this, -600.0, 0.5);
        }
    }
}
