package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainSubsystem {

    // ---- MOTORS ----
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    // ==== ENCODER / WHEEL CONSTANTS (METRIC) ====
    // goBILDA 435 RPM encoder (change if you're using a different motor)
    private static final double TICKS_PER_REV = 383.6;

    // Swyft Drive wheels: 86 mm diameter
    private static final double WHEEL_DIAMETER_MM = 86.0;
    private static final double WHEEL_CIRCUMFERENCE_MM = Math.PI * WHEEL_DIAMETER_MM;

    // Ticks per mm of *forward* travel
    private static final double TICKS_PER_MM = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_MM;

    // Ticks per mm of *strafe* — start same as forward and tune this if your strafe
    // distance is off (e.g. 1.1 * TICKS_PER_MM if needed).
    private static final double STRAFE_TICKS_PER_MM = TICKS_PER_MM;

    // TeleOp velocity cap (same as you had)
    private static final double MAX_TICKS_PER_SEC = 2815.0;

    // 1.0 = full speed, 0.4 = 40% speed, etc.
    private double driveScale = 1.0;

    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        // --- HARDWARE MAPPING ---
        frontLeft  = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "BackRight");

        // --- MOTOR DIRECTION ---
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- MOTOR BEHAVIOR ---
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Encoders for teleop
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ====== TELEOP MECANUM DRIVE (unchanged) ======
    public void drive(double leftX, double leftY, double rightX) {
        // Mecanum Drive using Swyft Drive V2
        double y = leftY;      // forward/back
        double x = -leftX;     // strafe
        double rx = -rightX;   // rotation

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        frontLeft.setVelocity(fl * MAX_TICKS_PER_SEC * driveScale);
        backLeft.setVelocity(bl * MAX_TICKS_PER_SEC * driveScale);
        frontRight.setVelocity(fr * MAX_TICKS_PER_SEC * driveScale);
        backRight.setVelocity(br * MAX_TICKS_PER_SEC * driveScale);
    }

    public void setDriveScale(double scale) {
        driveScale = scale;
    }

    public double getDriveScale() {
        return driveScale;
    }

    // ====== AUTO HELPERS (METRIC) ======

    /**
     * Drive straight forward (positive distance) or backward (negative distance)
     * using encoders. Distance is in millimeters.
     *
     * @param opMode     pass "this" from a LinearOpMode
     * @param distanceMm distance in mm (+ forward, - backward)
     * @param power      0.0–1.0 (RUN_TO_POSITION power)
     */
    public void driveForwardMm(LinearOpMode opMode, double distanceMm, double power) {
        // Convert distance to encoder ticks
        int moveTicks = (int) Math.round(distanceMm * TICKS_PER_MM);

        int flTarget = frontLeft.getCurrentPosition() + moveTicks;
        int frTarget = frontRight.getCurrentPosition() + moveTicks;
        int blTarget = backLeft.getCurrentPosition() + moveTicks;
        int brTarget = backRight.getCurrentPosition() + moveTicks;

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Clamp power
        power = Math.max(0.0, Math.min(1.0, power));

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        // Wait for movement to finish
        while (opMode.opModeIsActive()
                && (frontLeft.isBusy() || frontRight.isBusy()
                || backLeft.isBusy() || backRight.isBusy())) {
            // optional telemetry
        }

        // Stop motors
        stopAll();

        // Back to normal mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Strafe right (positive distance) or left (negative distance) using encoders.
     * Distance is in millimeters.
     *
     * @param opMode     pass "this" from a LinearOpMode
     * @param distanceMm +mm = strafe RIGHT, -mm = strafe LEFT
     * @param power      0.0–1.0 (RUN_TO_POSITION power)
     */
    public void strafeMm(LinearOpMode opMode, double distanceMm, double power) {
        // Convert distance to encoder ticks for strafe
        int moveTicks = (int) Math.round(distanceMm * STRAFE_TICKS_PER_MM);

        // For mecanum pure strafe:
        //  FL: +ticks, FR: -ticks, BL: -ticks, BR: +ticks (for strafe right)
        int flTarget = frontLeft.getCurrentPosition() + moveTicks;
        int frTarget = frontRight.getCurrentPosition() - moveTicks;
        int blTarget = backLeft.getCurrentPosition() - moveTicks;
        int brTarget = backRight.getCurrentPosition() + moveTicks;

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Clamp power
        power = Math.max(0.0, Math.min(1.0, power));

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        // Wait for movement to finish
        while (opMode.opModeIsActive()
                && (frontLeft.isBusy() || frontRight.isBusy()
                || backLeft.isBusy() || backRight.isBusy())) {
            // optional telemetry
        }

        // Stop motors
        stopAll();

        // Back to normal mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Rotate in place using time (no encoders).
     * Positive power: one direction, Negative: opposite.
     * @param opMode  pass "this" from a LinearOpMode
     * @param power   -1.0 to 1.0 (use something like 0.3–0.5)
     * @param ms      duration in milliseconds
     */
    public void rotateInPlaceForMs(LinearOpMode opMode, double power, long ms) {
        // Clamp power for safety
        power = Math.max(-1.0, Math.min(1.0, power));

        double vel = power * MAX_TICKS_PER_SEC;

        // Left side forward, right side backward for in-place rotation
        frontLeft.setVelocity( vel);
        backLeft.setVelocity( vel);
        frontRight.setVelocity(-vel);
        backRight.setVelocity(-vel);

        opMode.sleep(ms);

        stopAll();
    }

}
