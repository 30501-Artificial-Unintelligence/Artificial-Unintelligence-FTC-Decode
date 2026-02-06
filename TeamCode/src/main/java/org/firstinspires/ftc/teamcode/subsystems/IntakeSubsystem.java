package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class IntakeSubsystem {
    private final CRServo leftServo;
    private final CRServo rightServo;

    // User setting
    private double intakeSpeed = 1.0;

    // Slew config: 0 -> 1 in 0.5s  => 2.0 power units per second
    public static double SLEW_TIME_0_TO_1_S = 1.0;
    private static double maxRatePerSec() {
        return 1.0 / SLEW_TIME_0_TO_1_S;
    }

    // Slew state
    private double targetPower = 0.0;
    private double currentPower = 0.0;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTimeSec;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "turretServoA");
        rightServo = hardwareMap.get(CRServo.class, "turretServoB");

        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.FORWARD);

        timer.reset();
        lastTimeSec = timer.seconds();

        // start at 0 power
        applyPower(0.0);
    }

    /** Call this EVERY loop to apply slew limiting */
    public void update() {
        double now = timer.seconds();
        double dt = now - lastTimeSec;
        lastTimeSec = now;

        if (dt <= 0) return;

        double maxDelta = maxRatePerSec() * dt;
        currentPower = moveTowards(currentPower, targetPower, maxDelta);

        applyPower(currentPower);
    }

    /** Slew-limited stop */
    public void stopIntake() {
        setTargetPower(0.0);
    }

    /** Slew-limited start */
    public void startIntake() {
        setTargetPower(intakeSpeed);
    }

    /** Slew-limited set based on gamepad input */
    public void intakeSetPower(double gamepadInput) {
        double intakePower = 0.5 + gamepadInput * 0.5; // 0.5..1.0
        setTargetPower(intakePower);
    }

    /** Optional helper if you want direct targets elsewhere */
    public void setTargetPower(double power) {
        targetPower = Range.clip(power, -1.0, 1.0);
    }

    public void setIntakeSpeed(double speed) {
        intakeSpeed = Range.clip(speed, -1.0, 1.0);
    }

    private void applyPower(double pwr) {
        leftServo.setPower(pwr);
        rightServo.setPower(pwr);
    }

    private static double moveTowards(double current, double target, double maxDelta) {
        if (target > current) return Math.min(current + maxDelta, target);
        return Math.max(current - maxDelta, target);
    }
}
