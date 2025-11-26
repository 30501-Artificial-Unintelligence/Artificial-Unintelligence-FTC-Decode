package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Spindexer {
    // ======= Ball colors =======
    public enum ballColor { NONE, PURPLE, GREEN}

    // ======= CONST =======
    private static final int numSlots = 3;
    private static final double degPerSlot = 360.0 / numSlots;
    private static final double intakeAngle = 0.0;
    private static final double turretAngle = 180.0;
    private static final double servoSPDXangle = 2.0; // 2:1

    // ======= VAR =======
    private ballColor[] slots = new ballColor[numSlots];    // color of each slot
    private int intakeSlotIndex = 0;                        // current slot facing intake
    private double spindexerAngleDeg = 0.0;                 // physical spindexer angle

    // ======= HARDWARE =======
    private Servo servo;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    // ======= CONSTRUCTOR =======
    public Spindexer(HardwareMap hardwareMap, Servo servo, ColorSensor colorSensor) {
        this.servo = servo;
        this.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        initSpindexer();

    }

    // ======= INITIALIZATION =======
    private void initSpindexer() {
        //Start spindexer at 0 deg
        setServoFromSpindexerAngle(0);

        //reset state
        spindexerAngleDeg = 0;
        intakeSlotIndex = 0;

        for (int i = 0; i < numSlots; i++) {
            slots[i] = ballColor.NONE;
        }
    }

    public void rotateSpindexer(double deg) {
        spindexerAngleDeg = normalizeAngle( spindexerAngleDeg + deg );

        //convert
        setServoFromSpindexerAngle(spindexerAngleDeg);

        //update intake slot index
        double slotsMoved = deg / degPerSlot;
        intakeSlotIndex = normalizeSlotIndex(
                intakeSlotIndex + Math.round(slotsMoved)
        );
    }

    public void detectBall() {
        if (!ballPresent()) return;

        ballColor detect = detectColor();

        slots[intakeSlotIndex] = detect;

        rotateSpindexer(degPerSlot);
    }

    private boolean ballPresent() {
        double distMM = distanceSensor.getDistance(DistanceUnit.MM);

        boolean close = distMM > 25 && distMM < 60;
        if (!close) return false;

        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int brightness = r + g + b;

        //shadow detection
        return brightness >= 80 && brightness <= 2000;
    }

    private ballColor detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        double total = r + g + b;
        if (total < 1) return ballColor.NONE;

        double rn = r / total;
        double gn = g / total;
        double bn = b / total;

        //Purple = high R+B, Low G
        if ((rn + bn) > 0.6 && gn < 0.2) {
            return ballColor.PURPLE;
        }
        //Green = high G, low R+B
        if (gn > 0.6 && (rn + bn) < 0.2) {
            return ballColor.GREEN;
        }

        // Fallback to dominant channel
        return (gn > rn && gn > bn) ? ballColor.GREEN : ballColor.PURPLE;
    }

    // ======= LOAD BALL =======
    public boolean loadBall(ballColor desiredColor) {
        // find slot with desired color

        int targetSlot = findSlotWithColor(desiredColor);

        if (targetSlot == -1) {
            // If none found, use any ball
            targetSlot = findAnyBall();
            if (targetSlot == -1) {
                return false;
                // No balls?
            }
        }

        double targetAngle = angleForSlotAtTurret(targetSlot);

        rotateSpindexer(shortestRotationTo(targetAngle));

        slots[targetSlot] = ballColor.NONE;

        return true;
    }

    // ======= PATTERN =======
    public void followPattern(ballColor[] pattern) {
        for (ballColor desired : pattern) {
            boolean ok = loadBall(desired);

            if (!ok) {
                int any = findAnyBall();
                if (any != -1)
                    loadBall(slots[any]);
            }
        }
    }

    // ======= HELPER FUNCTIONS =======
    private int findSlotWithColor(ballColor c) {
        for (int i = 0; i < numSlots; i++) {
            if (slots[i] == c) return i;
        }
        return -1;
    }

    private int findAnyBall() {
        for (int i = 0; i < numSlots; i++) {
            if (slots[i] != ballColor.NONE) return i;
        }
        return -1;
    }

    private double angleForSlotAtTurret(int slot) {
        double slotAngle = slot * degPerSlot;
        return normalizeAngle(slotAngle + (turretAngle - intakeSlotIndex));

    }

    private double normalizeAngle(double deg) {
        while (deg < 0) deg += 360;
        while (deg >= 360) deg -= 360;
        return deg;
    }

    private int normalizeSlotIndex(long idx) {
        return (int)((idx % numSlots + numSlots) & numSlots);
    }

    private double shortestRotationTo(double targetAngle) {
        double diff = normalizeAngle(targetAngle - spindexerAngleDeg);
        if (diff > 180) diff -= 360;
        return diff;
    }

    private void setServoFromSpindexerAngle(double spdxAngle) {
        double servoDeg = spdxAngle * servoSPDXangle;
        double servoPos = servoDeg / 360.0;
        servo.setPosition(servoPos);
    }

}