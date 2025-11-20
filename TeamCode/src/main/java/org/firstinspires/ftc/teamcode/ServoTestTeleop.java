package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Servo Test", group = "Test")
public class ServoTestTeleop extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
            //initialization
            Servo servoOne;
            servoOne = hardwareMap.get(Servo.class, "servo_one");
            servoOne.setPosition(0.5);

            waitForStart();;
            while(opModeIsActive()){
                servoOne.setPosition(gamepad1.right_stick_y);
            }
    }
}

