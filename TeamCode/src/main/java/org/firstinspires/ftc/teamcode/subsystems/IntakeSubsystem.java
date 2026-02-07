package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class IntakeSubsystem {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final Servo stopperServo;

    // User setting
    public static double INTAKE_SPEED = 1.0;

    // Slew config: 0 -> 1 in this many seconds
    public static double SLEW_TIME_0_TO_1_S = 0.5;

    // === PANELS-TUNABLE STOPPER POSITIONS ===
    // Tune these in Panels (0..1)
    public static double STOPPER_OPEN_POS  = 0.2;
    public static double STOPPER_CLOSE_POS = 1.0;

    // If servo direction/geometry is reversed, flip this in Panels
    public static boolean STOPPER_INVERT = false;

    // When using setTargetPower directly, threshold for "running"
    public static double STOPPER_OPEN_THRESHOLD = 0.02;

    // Slew state
    private double targetPower = 0.0;
    private double currentPower = 0.0;

    private boolean stopperOpen = false;

    private long lastNs;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "turretServoA");
        rightServo = hardwareMap.get(CRServo.class, "turretServoB");
        stopperServo = hardwareMap.get(Servo.class, "servo_one");

        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.FORWARD);

        lastNs = System.nanoTime();

        applyPower(0.0);
        setStopperOpen(false); // start closed
    }

    /** Call this EVERY loop */
    public void update() {
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastNs) / 1e9;
        lastNs = nowNs;

        if (dt > 0) {
            double rate = (SLEW_TIME_0_TO_1_S <= 1e-6) ? 1e9 : (1.0 / SLEW_TIME_0_TO_1_S);
            double maxDelta = rate * dt;

            currentPower = moveTowards(currentPower, targetPower, maxDelta);
            applyPower(currentPower);
        }

        // OPTIONAL: live-apply updated panel values immediately to the servo
        // (so if you change STOPPER_OPEN_POS/CLOSE_POS in Panels while it's open/closed,
        // it will move right away)
        applyStopperPositionForCurrentState();
    }

    public void stopIntake() {
        setTargetPower(0.0);
        setStopperOpen(false);
    }

    public void startIntake() {
        setTargetPower(INTAKE_SPEED);
        setStopperOpen(true);
    }

    public void intakeSetPower(double gamepadInput) {
        double intakePower = 0.5 + gamepadInput * 0.5; // 0..1
        setTargetPower(intakePower);
    }

    public void setTargetPower(double power) {
        targetPower = Range.clip(power, -1.0, 1.0);
        setStopperOpen(Math.abs(targetPower) > STOPPER_OPEN_THRESHOLD);
    }

    private void setStopperOpen(boolean open) {
        if (open == stopperOpen) return;
        stopperOpen = open;
        applyStopperPositionForCurrentState();
    }

    private void applyStopperPositionForCurrentState() {
        double openPos  = STOPPER_INVERT ? STOPPER_CLOSE_POS : STOPPER_OPEN_POS;
        double closePos = STOPPER_INVERT ? STOPPER_OPEN_POS  : STOPPER_CLOSE_POS;

        double pos = stopperOpen ? openPos : closePos;
        stopperServo.setPosition(Range.clip(pos, 0.0, 1.0));
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
