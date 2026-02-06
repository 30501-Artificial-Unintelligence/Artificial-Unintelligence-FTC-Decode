package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * PURE Brushland digital pin tester.
 * Shows ONLY the boolean states of the 4 digital channels:
 *  - slot1Present, slot1Green, slot2Present, slot2Green
 *
 * Make sure these names match your Robot Configuration exactly.
 */
@TeleOp(name = "TEST Brushland Digital Pins", group = "Test")
public class TestBrushlandDigitalPins extends OpMode {

    private DigitalChannel slot1Present, slot1Green, slot2Present, slot2Green;

    @Override
    public void init() {
        slot1Present = hardwareMap.get(DigitalChannel.class, "slot1Present");
        slot1Green   = hardwareMap.get(DigitalChannel.class, "slot1Green");
        slot2Present = hardwareMap.get(DigitalChannel.class, "slot2Present");
        slot2Green   = hardwareMap.get(DigitalChannel.class, "slot2Green");

        slot1Present.setMode(DigitalChannel.Mode.INPUT);
        slot1Green.setMode(DigitalChannel.Mode.INPUT);
        slot2Present.setMode(DigitalChannel.Mode.INPUT);
        slot2Green.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Brushland Digital Pins Test");
        telemetry.addLine("Displays raw true/false of each channel.");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("slot1Present", slot1Present.getState());
        telemetry.addData("slot1Green",   slot1Green.getState());
        telemetry.addData("slot2Present", slot2Present.getState());
        telemetry.addData("slot2Green",   slot2Green.getState());
        telemetry.update();
    }
}

