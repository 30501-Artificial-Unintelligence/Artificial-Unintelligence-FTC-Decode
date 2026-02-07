package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.bylazar.configurables.annotations.Configurable;

@TeleOp(name = "Servo Test2", group = "Test")
public class ServoTestTeleop2 extends LinearOpMode {

    private double servoPosition = 0.5; // start centered

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoOne = hardwareMap.get(Servo.class, "servo_one");

        servoOne.setPosition(servoPosition);

        waitForStart();

        while (opModeIsActive()) {
            // Up is negative on the stick, so invert it
            double stick = -gamepad1.right_stick_y; // now up is positive

            // Map stick [-1, 1] -> servo [0, 1]
            servoPosition = 0.5 + stick * 0.5;
            servoPosition = Range.clip(servoPosition, 0.0, 1.0);

            servoOne.setPosition(servoPosition);

            telemetry.addData("Servo Position Cmd", servoPosition);
            telemetry.addData("Stick", stick);
            telemetry.update();

            //idle();
        }
    }
}
