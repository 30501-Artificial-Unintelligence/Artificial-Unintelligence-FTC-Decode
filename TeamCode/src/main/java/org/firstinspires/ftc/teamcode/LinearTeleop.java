package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "First_Teleop")
public class LinearTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //intialization code
        DcMotor motorOne;
        motorOne = hardwareMap.get(DcMotor.class,"motor_one");

        motorOne.setDirection(DcMotorSimple.Direction.FORWARD);

        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //resets encoder ticks

        waitForStart();

        while(opModeIsActive()){
            motorOne.setPower(1); //[-1,1] // for DCmotor , non Ex
            //motorOne.setVelocity(2800);
        }
    }
}
