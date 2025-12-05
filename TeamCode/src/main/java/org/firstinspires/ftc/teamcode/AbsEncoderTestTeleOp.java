package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Abs Encoder Test", group = "Test")
public class AbsEncoderTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        AnalogInput abs = hardwareMap.get(AnalogInput.class, "spindexerAbs");

        telemetry.addLine("Rotate the encoder by hand and watch the voltage & deg.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double v = abs.getVoltage();
            double deg = (v / 3.3) * 360.0;

            telemetry.addData("Voltage", "%.3f V", v);
            telemetry.addData("Angle", "%.1f deg", deg);
            telemetry.update();
        }
    }
}
