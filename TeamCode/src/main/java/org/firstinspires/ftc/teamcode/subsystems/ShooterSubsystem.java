package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ShooterSubsystem {

    // ===== MOTOR CONSTANTS =====
    // goBILDA 6000 RPM motor
    private static final double TICKS_PER_REV = 28.0;
    private static final double PHYSICAL_MAX_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;
    private static final double MAX_RPM = PHYSICAL_MAX_RPM;

    // ===== HOOD SERVO CONSTANTS =====
    // Tuned from your servo test:
    //  0.85 -> ~35 degrees (near)
    //  0.92 -> ~60 degrees (far-ish)
    private static final double HOOD_NEAR_POS = 0.85;  // adjust in practice
    private static final double HOOD_FAR_POS  = 0.92;  // adjust in practice

    // ===== HELPERS =====
    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    // ===== HARDWARE =====
    private final DcMotorEx motor;
    private final Servo hoodServo;

    // ===== STATE =====

    // True if the subsystem is currently commanded to run
    private boolean isOn = false;

    // 0 = nearfield, 1 = far
    private int fieldPos = 0;

    // independent RPMs for near and far
    private double nearRpm = 2700.0;  // tune these on-field
    private double farRpm  = 3300.0;

    // edge-detection for RPM buttons (TeleOp sends raw button states)
    private boolean prevRpmUp   = false;
    private boolean prevRpmDown = false;

    // ===== PID STATE =====
    private double kP = 0.0008;
    private double kI = 0.0;
    private double kD = 0.0002;

    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private boolean pidInitialized = false;

    private final ElapsedTime pidTimer = new ElapsedTime();


    public ShooterSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_one");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Start in NEAR field by default
        fieldPos = 0;
        hoodServo.setPosition(HOOD_NEAR_POS);
    }

    /**
     * Call this once per loop from your TeleOp.
     *
     * @param shooterOnCommand  True = run shooter, False = stop (comes from your TeleOp logic).
     * @param rpmUpButton       e.g. gamepad1.dpad_up (rising edge bumps RPM for current fieldPos).
     * @param rpmDownButton     e.g. gamepad1.dpad_down (rising edge lowers RPM for current fieldPos).
     * @param fieldPosInput     0 = near, 1 = far (comes from your TeleOp / auto logic).
     */
    public void update(boolean shooterOnCommand,
                       boolean rpmUpButton,
                       boolean rpmDownButton,
                       int fieldPosInput) {

        // === FIELD POSITION / HOOD CONTROL ===
        // Clamp to [0,1] just in case
        int newFieldPos = (fieldPosInput == 1) ? 1 : 0;

        if (newFieldPos != fieldPos) {
            fieldPos = newFieldPos;

            // Move hood when mode changes
            if (fieldPos == 1) {
                // far field
                hoodServo.setPosition(HOOD_FAR_POS);
            } else {
                // near field
                hoodServo.setPosition(HOOD_NEAR_POS);
            }
        }

        // === RPM ADJUSTMENT (per mode) ===
        // Rising-edge detection so holding the button doesn't spam
        if (rpmUpButton && !prevRpmUp) {
            if (fieldPos == 1) {
                // far field
                farRpm += RPM_STEP;
                if (farRpm > PHYSICAL_MAX_RPM) farRpm = PHYSICAL_MAX_RPM;
            } else {
                // near field
                nearRpm += RPM_STEP;
                if (nearRpm > PHYSICAL_MAX_RPM) nearRpm = PHYSICAL_MAX_RPM;
            }
        }

        if (rpmDownButton && !prevRpmDown) {
            if (fieldPos == 1) {
                // far field
                farRpm -= RPM_STEP;
                if (farRpm < 0) farRpm = 0;
            } else {
                // near field
                nearRpm -= RPM_STEP;
                if (nearRpm < 0) nearRpm = 0;
            }
        }

        // Save button states for next loop
        prevRpmUp = rpmUpButton;
        prevRpmDown = rpmDownButton;

        // === APPLY SHOOTER COMMAND ===
        isOn = shooterOnCommand;

        double targetRpm = (fieldPos == 1) ? farRpm : nearRpm;
        if (isOn && targetRpm > 0) {
            runPidStep(targetRpm);
        } else {
            stop(); // ensures power = 0 and PID reset
        }
    }

    // ==== PID UPDATE ====
    private void runPidStep(double targetRpm) {
        double now = pidTimer.seconds();

        if (!pidInitialized) {
            lastTime = now;
            lastError = 0.0;
            integral = 0.0;
            pidInitialized = true;
        }
        double dt = now - lastTime;
        if (dt <= 0) {
            return;
        }

        double currentRpm = getCurrentRpmEstimate();
        double error = targetRpm - currentRpm;

        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double pid = kP * error + kI * integral + kD * derivative;
        // Super simple feedforward: "how fast do we want, as a fraction of max"
        double ff = targetRpm / MAX_RPM;

        double power = ff + pid;

        // Make sure power is legal
        power = Range.clip(power, 0.0, 1.0);

        motor.setPower(power);

        lastError = error;
        lastTime = now;
    }

    // ===== GETTERS FOR TELEMETRY / OTHER SUBSYSTEMS =====

    public boolean isOn() {
        return isOn;
    }

    /** 0 = near, 1 = far */
    public int getFieldPos() {
        return fieldPos;
    }

    public double getNearRpm() {
        return nearRpm;
    }

    public double getFarRpm() {
        return farRpm;
    }

    /** Target RPM for current field position. */
    public double getTargetRpm() {
        return (fieldPos == 1) ? farRpm : nearRpm;
    }

    /** Estimated current RPM from encoder velocity. */
    public double getCurrentRpmEstimate() {
        double ticksPerSec = motor.getVelocity(); // encoder velocity in ticks/sec
        return ticksPerSecToRpm(ticksPerSec);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    public void stop() {
        isOn = false;
        motor.setPower(0.0);
    }

    public  void setNearRPM (long RPM){
        nearRpm = RPM;
    }

    public void setFarRPM (long RPM){
        farRpm = RPM;
    }
}
