package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private final DcMotor motorOne;
    private boolean isOn = false;
    private boolean lastButton = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {

        motorOne = hardwareMap.get(DcMotor.class,"motor_one");

        motorOne.setDirection(DcMotorSimple.Direction.FORWARD);

        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder ticks
    }

    public void update(boolean buttonPressed) {
        // Edge detection: button just went from not pressed -> pressed
        if (buttonPressed && !lastButton) {
            isOn = !isOn;
            motorOne.setPower(isOn ? 1.0 : 0.0);
        }

        lastButton = buttonPressed;
    }

    public boolean isOn() {
        return isOn;
    }

    public void stop() {
        isOn = false;
        motorOne.setPower(0.0);
    }
}
