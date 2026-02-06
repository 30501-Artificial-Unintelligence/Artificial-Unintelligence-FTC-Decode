package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.teamcode.subsystems.util.ColorRangefinder;

@TeleOp
public class ConfigureBrushlandSlotSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        ColorRangefinder crf = new ColorRangefinder(
                hardwareMap.get(RevColorSensorV3.class, "flash") // rename to your config
        );

        // ======= TUNE THESE =======
        double PRESENT_MM = 25; // distance requirement
        double GREEN_LO = 110;  // hue degrees
        double GREEN_HI = 140;  // hue degrees

        waitForStart();

        // Pin0: PRESENCE (distance only)
        crf.setPin0Digital(ColorRangefinder.DigitalMode.DISTANCE, PRESENT_MM, PRESENT_MM);

        // Pin1: GREEN DETECT (HSV hue window)
        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, GREEN_LO / 360.0 * 255, GREEN_HI / 360.0 * 255);

        // Add the SAME distance gate to Pin1 so it only triggers when close
        crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.DISTANCE, PRESENT_MM);

        // Optional: LED brightness (helps consistency)
        crf.setLedBrightness(180);

        // Done
        while (opModeIsActive()) {
            telemetry.addData("Configured", true);
            telemetry.update();
            sleep(100);
        }
    }


}
