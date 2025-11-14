package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//This test works
@TeleOp (name = "First_Teleop")
public class LinearTeleop extends LinearOpMode {
    private ShooterSubsystem shooter;


    @Override
    public void runOpMode() throws InterruptedException {
        //intialization code
        //init hardware via subsystem
        shooter = new ShooterSubsystem(hardwareMap);


        //init Vars


        waitForStart();

        while(opModeIsActive()){
            shooter.update(gamepad1.a);

            telemetry.addData("Shooter on", shooter.isOn());
            telemetry.update();
        }
    }
}
