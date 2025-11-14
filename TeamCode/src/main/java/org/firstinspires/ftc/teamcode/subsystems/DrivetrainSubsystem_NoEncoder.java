package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainSubsystem_NoEncoder {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Optional: global speed scale (1.0 = full speed)
    private double driveScale = 1.0;

    public DrivetrainSubsystem_NoEncoder(HardwareMap hardwareMap) {
        // --- HARDWARE MAPPING ---
        frontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight  = hardwareMap.get(DcMotor.class, "BackRight");

        // --- MOTOR DIRECTION ---
        // Common convention: reverse the RIGHT side
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- MOTOR BEHAVIOR ---
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- RUN WITHOUT ENCODERS ---
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Main drive method.
     * @param leftX   gamepad1.left_stick_x   (strafe)
     * @param leftY   gamepad1.left_stick_y   (forward/back)
     * @param rightX  gamepad1.right_stick_x  (rotate)
     */
    public void driveNoEncoder(double leftX, double leftY, double rightX) {
        double y  = -leftY;   // forward/back (stick up = forward)
        double x  =  leftX;   // strafe
        double rx =  rightX;  // rotation

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        // apply speed scale and set power
        frontLeft.setPower(fl * driveScale);
        backLeft.setPower(bl * driveScale);
        frontRight.setPower(fr * driveScale);
        backRight.setPower(br * driveScale);
    }

    // Optional: slow mode / scaling
    public void setDriveScale(double scale) {
        driveScale = scale;
    }

    public double getDriveScale() {
        return driveScale;
    }
}
