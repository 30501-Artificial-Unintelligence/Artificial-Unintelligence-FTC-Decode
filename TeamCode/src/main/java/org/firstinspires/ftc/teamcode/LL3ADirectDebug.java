package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name = "LL3A Direct Debug", group = "Test")
public class LL3ADirectDebug extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Get the Limelight from the Robot Config (device name must be "limelight")
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Poll rate and pipeline index (use the AprilTag pipeline index you actually use)
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(8);   // <-- change 8 if your AprilTag pipeline is different

        telemetry.addLine("LL3A Direct Debug INIT");
        telemetry.addLine("Make sure pipeline 8 is an AprilTag pipeline.");
        telemetry.update();

        waitForStart();

        // Start streaming AFTER start so we know this OpMode owns it
        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            boolean hasResult = (result != null);
            telemetry.addData("HasResult", hasResult);

            int pattern = 0; // 0, 21, 22, 23 only

            if (hasResult) {
                telemetry.addData("IsValid", result.isValid());

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                int tagCount = (tags == null) ? -1 : tags.size();
                telemetry.addData("TagCount", tagCount);

                if (tags != null && !tags.isEmpty()) {
                    LLResultTypes.FiducialResult firstTag = tags.get(0);
                    int id = firstTag.getFiducialId();
                    telemetry.addData("FirstTagID", id);

                    // Compute "pattern" based on ID
                    switch (id) {
                        case 21:
                        case 22:
                        case 23:
                            pattern = id;
                            break;
                        default:
                            pattern = 0; // not one of our three
                            break;
                    }
                } else {
                    telemetry.addData("FirstTagID", "none");
                }
            }

            telemetry.addData("Pattern", pattern);  // 0 / 21 / 22 / 23
            telemetry.update();

            sleep(50);
        }

        // Cleanly stop camera when OpMode ends
        limelight.stop();
    }
}
