package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.util.PIDFController;

@Configurable
public class ShooterSubsystemFF {

    // ===== MOTOR CONSTANTS =====
    private static final double TICKS_PER_REV = 28.0;      // goBILDA 6000 motor encoder
    private static final double PHYSICAL_MAX_RPM = 6000.0;
    private static final double RPM_STEP = 250.0;

    private static final double MAX_TICKS_PER_SEC = PHYSICAL_MAX_RPM * TICKS_PER_REV / 60.0;

    // ===== HOOD SERVO CONSTANTS =====
    private static final double HOOD_NEAR_POS = 0.85;
    private static final double HOOD_FAR_POS  = 0.85;

    // =========================
    // PANELS TUNABLES (STATIC)
    // =========================
    // These show up in Panels Configurables (must be public static + non-final)

    // RPM setpoints (human-friendly)
    public static double TUNE_NEAR_RPM = 2600.0;
    public static double TUNE_FAR_RPM  = 3200.0;

    // Hood positions (optional to tune)
    public static double TUNE_HOOD_NEAR_POS = HOOD_NEAR_POS;
    public static double TUNE_HOOD_FAR_POS  = HOOD_FAR_POS;

    // PID + FF tuned like your FlywheelTuning:
    // FF = kV * targetVelocity(ticks/sec) + kS
    public static double TUNE_kP = 0.012;
    public static double TUNE_kI = 0.0;
    public static double TUNE_kD = 0.0;
    public static double TUNE_kV = 0.00042; // good first guess ~ 1/MAX_TICKS_PER_SEC
    public static double TUNE_kS = 0.05;

    public static boolean TUNE_FORCE_ON = false;
    public static double  TUNE_FORCE_RPM = 3000.0; // any rpm you want while tuning


    // ===== HARDWARE =====
    private final DcMotorEx motor;
    private final Servo hoodServo;
    private final LightSubsystem light;

    // ===== CONTROLLER =====
    private final PIDFController controller;

    // ===== STATE =====
    private boolean isOn = false;
    private int fieldPos = 0; // 0 near, 1 far

    private double nearRpm = 3050.0;
    private double farRpm  = 3700.0;

    private boolean prevRpmUp   = false;
    private boolean prevRpmDown = false;

    // ===== TUNABLES (INSTANCE COPIES; keep your original methods working) =====
    // Units:
    //  - controller error is ticks/sec (target - measured)
    //  - kV is power per (ticks/sec)
    //  - kS is static power offset
    private double kP = 0.00000;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kV = 0.0;
    private double kS = 0.00;

    // debug
    private double lastTargetTps = 0.0;
    private double lastVelTps = 0.0;
    private double lastPowerCmd = 0.0;
    private double lastFF = 0.0;

    public ShooterSubsystemFF(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_one");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        light = new LightSubsystem(hardwareMap);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // We are doing our own velocity loop -> run open-loop power
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize statics if you want an auto-guess for kV
//        if (TUNE_kV == 0.0) {
//            TUNE_kV = 1.0 / MAX_TICKS_PER_SEC;
//        }

        // Sync instance from Panels tunables once on init
        syncFromTunables();

        controller = new PIDFController(kP, kI, kD, 0.0);

        fieldPos = 0;
        hoodServo.setPosition(TUNE_HOOD_NEAR_POS);
    }

    private void syncFromTunables() {
        // pull Panels -> instance
        nearRpm = TUNE_NEAR_RPM;
        farRpm  = TUNE_FAR_RPM;

        kP = TUNE_kP;
        kI = TUNE_kI;
        kD = TUNE_kD;
        kV = TUNE_kV;
        kS = TUNE_kS;
    }

    private void syncToTunables() {
        // push instance -> Panels (note: Panels UI may not reflect until refresh, but code stays consistent)
        TUNE_NEAR_RPM = nearRpm;
        TUNE_FAR_RPM  = farRpm;

        TUNE_kP = kP;
        TUNE_kI = kI;
        TUNE_kD = kD;
        TUNE_kV = kV;
        TUNE_kS = kS;
    }

    private static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    public void update(boolean shooterOnCommand,
                       boolean rpmUpButton,
                       boolean rpmDownButton,
                       int fieldPosInput) {

        // Always pull latest Panels values at start of loop
        syncFromTunables();

        // === FIELD POS / HOOD ===
        int newFieldPos = (fieldPosInput == 1) ? 1 : 0;
        if (newFieldPos != fieldPos) {
            fieldPos = newFieldPos;
            hoodServo.setPosition(fieldPos == 1 ? TUNE_HOOD_FAR_POS : TUNE_HOOD_NEAR_POS);
        }

        // === RPM ADJUST (edge) ===
        if (rpmUpButton && !prevRpmUp) {
            if (fieldPos == 1) farRpm = Math.min(PHYSICAL_MAX_RPM, farRpm + RPM_STEP);
            else               nearRpm = Math.min(PHYSICAL_MAX_RPM, nearRpm + RPM_STEP);
            syncToTunables();
        }
        if (rpmDownButton && !prevRpmDown) {
            if (fieldPos == 1) farRpm = Math.max(0.0, farRpm - RPM_STEP);
            else               nearRpm = Math.max(0.0, nearRpm - RPM_STEP);
            syncToTunables();
        }
        prevRpmUp = rpmUpButton;
        prevRpmDown = rpmDownButton;

        // === APPLY COMMAND ===
        boolean on = shooterOnCommand || TUNE_FORCE_ON;
        isOn = on;

        double targetRpm = TUNE_FORCE_ON ? TUNE_FORCE_RPM : getTargetRpm();
        double targetTps = (isOn && targetRpm > 0) ? rpmToTicksPerSec(targetRpm) : 0.0;

        double velTps = motor.getVelocity(); // ticks/sec even in RUN_WITHOUT_ENCODER
        double errorTps = targetTps - velTps;

        lastTargetTps = targetTps;
        lastVelTps = velTps;

        if (targetTps <= 1.0) {
            // OFF
            motor.setPower(0.0);
            lastPowerCmd = 0.0;
            lastFF = 0.0;
            light.setColor(1);
            return;
        }

        // Feedforward (FlywheelTuning style):
        // FF = kV * targetVelocity(ticks/sec) + kS
        double ff = (kV * targetTps) + kS;
        lastFF = ff;

        // Push params into controller each loop (so tuning updates instantly)
        controller.setPIDF(kP, kI, kD, ff);

        // Controller output is power
        double power = controller.calculate(errorTps);
        power = Range.clip(power, -1.0, 1.0);

        motor.setPower(power);
        lastPowerCmd = power;

        // Light logic based on RPM error
        double curRpm = ticksPerSecToRpm(velTps);
        double errRpm = getTargetRpm() - curRpm;

        if (Math.abs(errRpm) < 200) light.setColor(3);
        else light.setColor(2);
    }

    // ===== GETTERS =====
    public boolean isOn() { return isOn; }
    public int getFieldPos() { return fieldPos; }
    public double getNearRpm() { return nearRpm; }
    public double getFarRpm() { return farRpm; }
    public double getTargetRpm() { return (fieldPos == 1) ? farRpm : nearRpm; }

    public double getTargetTps() { return lastTargetTps; }
    public double getVelocityTps() { return lastVelTps; }
    public double getVelocityRpm() { return ticksPerSecToRpm(lastVelTps); }
    public double getPowerCmd() { return lastPowerCmd; }
    public double getFF() { return lastFF; }

    public double getKp() { return kP; }
    public double getKi() { return kI; }
    public double getKd() { return kD; }
    public double getKv() { return kV; }
    public double getKs() { return kS; }

    public void adjustKp(double d) { kP = Math.max(0.0, kP + d); syncToTunables(); }
    public void adjustKi(double d) { kI = Math.max(0.0, kI + d); syncToTunables(); }
    public void adjustKd(double d) { kD = Math.max(0.0, kD + d); syncToTunables(); }
    public void adjustKv(double d) { kV = Math.max(0.0, kV + d); syncToTunables(); }
    public void adjustKs(double d) { kS = kS + d; syncToTunables(); } // allow negative for sag sim

    public void stop() {
        isOn = false;
        motor.setPower(0.0);
        light.setColor(1);
    }

    public double getCurrentRpmEstimate() {
        double ticksPerSec = motor.getVelocity();
        return ticksPerSecToRpm(ticksPerSec);
    }
}
