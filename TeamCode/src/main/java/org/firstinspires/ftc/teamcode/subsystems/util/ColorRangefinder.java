package org.firstinspires.ftc.teamcode.subsystems.util;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Brushland Labs Color Rangefinder configuration helper.
 * Docs: Brushland Labs "Color Rangefinder" configuration.
 */
public class ColorRangefinder {
    private final I2cDeviceSynchSimple i2c;

    // other writable registers
    private static final byte CALIB_A_VAL_0   = 0x32;
    private static final byte PS_DISTANCE_0   = 0x42;
    private static final byte LED_BRIGHTNESS  = 0x46;
    private static final byte I2C_ADDRESS_REG = 0x47;

    private enum PinNum {
        PIN0(0x28), PIN1(0x2D);
        private final byte modeAddress;
        PinNum(int modeAddress) { this.modeAddress = (byte) modeAddress; }
    }

    public enum DigitalMode {
        RED(1), BLUE(2), GREEN(3), ALPHA(4), HSV(5), DISTANCE(6);
        public final byte value;
        DigitalMode(int value) { this.value = (byte) value; }
    }

    public enum AnalogMode {
        RED(13), BLUE(14), GREEN(15), ALPHA(16), HSV(17), DISTANCE(18);
        public final byte value;
        AnalogMode(int value) { this.value = (byte) value; }
    }

    public ColorRangefinder(RevColorSensorV3 emulator) {
        // Brushland emulates Rev Color Sensor V3 over I2C
        this.i2c = emulator.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);

        // Optional: set I2C FAST mode if supported (400kHz)
        try {
            if (i2c instanceof LynxI2cDeviceSynch) {
                ((LynxI2cDeviceSynch) i2c).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            }
        } catch (Exception ignored) { }
    }

    // ----- Pin 0 helpers -----
    public void setPin0Digital(DigitalMode mode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN0, mode, lowerBound, higherBound);
    }

    public void setPin0DigitalMaxDistance(DigitalMode mode, double mmRequirement) {
        // Brushland docs show passing HSV here; this is how they store the distance requirement. :contentReference[oaicite:1]{index=1}
        setPin0Digital(mode, mmRequirement, mmRequirement);
    }

    public void setPin0Analog(AnalogMode mode, int denominator) {
        byte denom0 = (byte) (denominator & 0xFF);
        byte denom1 = (byte) ((denominator & 0xFF00) >> 8);
        i2c.write(PinNum.PIN0.modeAddress, new byte[]{mode.value, denom0, denom1});
    }

    public void setPin0Analog(AnalogMode mode) {
        setPin0Analog(mode, mode == AnalogMode.DISTANCE ? 100 : 0xFFFF);
    }

    // ----- Pin 1 helpers -----
    public void setPin1Digital(DigitalMode mode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN1, mode, lowerBound, higherBound);
    }

    public void setPin1DigitalMaxDistance(DigitalMode mode, double mmRequirement) {
        setPin1Digital(mode, mmRequirement, mmRequirement);
    }

    public void setLedBrightness(int value0to255) {
        i2c.write8(LED_BRIGHTNESS, value0to255);
    }

    public void setI2cAddress(int addr1to127) {
        i2c.write8(I2C_ADDRESS_REG, addr1to127 << 1);
    }

    public float[] getCalibration() {
        ByteBuffer bytes = ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0, 16)).order(ByteOrder.LITTLE_ENDIAN);
        return new float[]{bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat()};
    }

    public double readDistanceMm() {
        ByteBuffer bytes = ByteBuffer.wrap(i2c.read(PS_DISTANCE_0, 4)).order(ByteOrder.LITTLE_ENDIAN);
        return bytes.getFloat();
    }

    // ---- internal write helper ----
    private void setDigital(PinNum pin, DigitalMode mode, double lowerBound, double higherBound) {
        int lo, hi;

        // Brushland example uses this "equal bounds" pattern for distance requirements. :contentReference[oaicite:2]{index=2}
        if (lowerBound == higherBound) {
            lo = (int) lowerBound;
            hi = (int) higherBound;
        } else if (mode.value <= DigitalMode.HSV.value) {
            // color (0-255) -> 16-bit raw
            lo = (int) Math.round(lowerBound / 255.0 * 65535);
            hi = (int) Math.round(higherBound / 255.0 * 65535);
        } else {
            // distance in mm
            float[] calib = getCalibration();
            if (lowerBound < .5) hi = 2048;
            else hi = rawFromDistance(calib[0], calib[1], calib[2], calib[3], lowerBound);
            lo = rawFromDistance(calib[0], calib[1], calib[2], calib[3], higherBound);
        }

        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);

        i2c.write(pin.modeAddress, new byte[]{mode.value, lo0, lo1, hi0, hi1});

        // Only use this class in a *configuration opmode*, not in TeleOp loop.
        try { Thread.sleep(25); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    private double root(double n, double v) {
        double val = Math.pow(v, 1.0 / Math.abs(n));
        if (n < 0) val = 1.0 / val;
        return val;
    }

    private int rawFromDistance(float a, float b, float c, float x0, double mm) {
        return (int) (root(b, (mm - c) / a) + x0);
    }
}
