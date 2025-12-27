package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemPIDF;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp with pedro and pidf", group = "Test")
public class TeleOp_pedro_pidf extends OpMode {

    // ===== PEDRO FOLLOWER / DRIVE =====
    private Follower follower;
    public static Pose startingPose; // optional: can be set from auto
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;   // same meaning as old teleop (0.4 speed)

    // ===== SUBSYSTEMS =====
    private ShooterSubsystemPIDF shooter;
    private IntakeSubsystem_Motor intake;
    private SpindexerSubsystem spindexer;
    private LoaderSubsystem loader;
    private TurretSubsystem turret;

    // ===== DASHBOARD =====
    private FtcDashboard dashboard;

    // ===== STATE VARIABLES (from old ShooterTestTeleOpPidf, trimmed) =====
    private boolean lastY = false;       // for spindexer eject (gamepad1 Y edge)
    private boolean prev2a = false;      // gamepad2 A slowMode toggle

    private int fieldPos = 0;            // 0 = nearfield, 1 = far

    private boolean spindexerIsFull = false;

    private boolean shooterOn = false;
    private boolean intakeOn = false;

    // turret buttons (gamepad1)
    private boolean prevA, prevDpadL, prevDpadR;

    boolean readyForIntake = true;
    private long shooterSpinDownDeadline = 0;   // time (ms) until we turn shooter off
    private boolean lastEjecting = false;       // edge-detect on eject end

    private int driverPatternTag = 0;  // 0 = fastest, 21/22/23 = pattern
    private boolean prev2Up, prev2Right, prev2Left, prev2Down;

    private boolean prevLeftStick = false;  // for fieldPos toggle (gamepad1 left stick button)

    // rehome button edge tracking
    private boolean prevRehome = false;

    private boolean prev2LeftBumper = false;
    private boolean prev2RightBumper = false;

    @Override
    public void init() {
        // ===== INIT FOLLOWER =====
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Example lazy path: from current pose to (45, 98) with heading ~45°
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(45),
                                0.8
                        )
                )
                .build();

        // ===== INIT SUBSYSTEMS =====
        shooter   = new ShooterSubsystemPIDF(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem_Motor(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        turret    = new TurretSubsystem(hardwareMap);

        // ===== DASHBOARD TELEMETRY HOOKUP =====
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("TeleOp with Pedro + Shooter PIDF");
        telemetry.addLine("Controls match ShooterTestTeleOpPidf (drive now via Pedro).");
        telemetry.update();
    }

    @Override
    public void start() {
        // start Pedro teleop drive
        follower.startTeleopDrive();

        // home spindexer so slot 0 is at intake
        spindexer.homeToIntake();

        shooterSpinDownDeadline = 0;
    }

    @Override
    public void loop() {
        // ===== UPDATE PEDRO FOLLOWER =====
        follower.update();
        telemetryM.update();

        // ===== DRIVETRAIN (match old TeleOp: gamepad2 sticks + slow mode on gamepad2.a) =====
        double leftX  = gamepad2.left_stick_x;
        double leftY  = gamepad2.left_stick_y;
        double rightX = gamepad2.right_stick_x;

        boolean a2 = gamepad2.a;
        if (a2 && !prev2a) {
            slowMode = !slowMode;
        }
        prev2a = a2;

        double driveScale = slowMode ? 0.4 : 1.0;

        if (!automatedDrive) {
            // signs chosen to match typical mecanum conventions;
            // if your old DrivetrainSubsystem inverted differently, you can tweak here.
            follower.setTeleOpDrive(
                    -leftY * driveScale,
                    -leftX * driveScale,
                    -rightX * driveScale,
                    true // Robot-centric (like original)
            );
        }

        // ===== AUTOMATED PATH FOLLOWING (Pedro sample controls) =====
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // ===== SHOOTER FIELD POSITION TOGGLE (gamepad1 left stick button) =====
        boolean leftStickButton = gamepad1.left_stick_button;
        if (leftStickButton && !prevLeftStick) {
            fieldPos = (fieldPos == 0) ? 1 : 0; // toggle near/far
        }
        prevLeftStick = leftStickButton;

        // Shooter update uses shooterOn; shooterOn decided later based on spindexer state.
        shooter.update(
                shooterOn,
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                fieldPos
        );

        // ===== LOADER =====
        loader.updateLoader();

        // ===== INTAKE =====
        if (intakeOn) {
            intake.startIntake();
        } else {
            intake.stopIntake();
        }

        // ===== TURRET CONTROL (same as original: gamepad1) =====
        boolean a  = gamepad1.a;
        boolean dl = gamepad1.dpad_left;
        boolean dr = gamepad1.dpad_right;

        if (a && !prevA) turret.goToAngle(0.0);
        if (dl && !prevDpadL) turret.goToAngle(90.0);
        if (dr && !prevDpadR) turret.goToAngle(-90.0);

        prevA      = a;
        prevDpadL  = dl;
        prevDpadR  = dr;

        double stickX = gamepad1.right_stick_x;
        if (Math.abs(stickX) > 0.05) {
            turret.setManualPower(stickX * 0.4);
        } else {
            turret.setManualPower(0.0);
        }

        turret.update();

        // ===== SPINDEXER / PATTERN INPUT =====

        // Y on gamepad1 starts eject sequence
        boolean yEdge = gamepad1.y && !lastY;
        lastY = gamepad1.y;

        // Rehome spindexer on right stick button (gamepad1)
        boolean rehomeButton = gamepad1.right_stick_button;
        if (rehomeButton && !prevRehome) {
            spindexer.homeToIntake();
        }
        prevRehome = rehomeButton;

        // driver 2 chooses pattern tag with dpad:
        boolean dUp2    = gamepad2.dpad_up;
        boolean dRight2 = gamepad2.dpad_right;
        boolean dLeft2  = gamepad2.dpad_left;
        boolean dDown2  = gamepad2.dpad_down;

        if (dUp2 && !prev2Up) {
            driverPatternTag = 23;
        }
        if (dRight2 && !prev2Right) {
            driverPatternTag = 22;
        }
        if (dLeft2 && !prev2Left) {
            driverPatternTag = 21;
        }
        if (dDown2 && !prev2Down) {
            driverPatternTag = 0; // fastest / no pattern
        }

        prev2Up    = dUp2;
        prev2Right = dRight2;
        prev2Left  = dLeft2;
        prev2Down  = dDown2;

        spindexerIsFull = spindexer.update(telemetry, loader, yEdge, driverPatternTag);

        // --- Eject / shooter timing logic (unchanged) ---
        boolean ejecting   = spindexer.isEjecting();
        boolean hasAnyBall = spindexer.hasAnyBall();
        long now = System.currentTimeMillis();

        if (lastEjecting && !ejecting && !hasAnyBall) {
            shooterSpinDownDeadline = now + 2000;  // 2 s after last ball
        }
        lastEjecting = ejecting;

        boolean wantShooter;
        boolean wantIntake;

        if (spindexerIsFull) {
            wantShooter = true;
            wantIntake  = false;
        } else if (ejecting || now < shooterSpinDownDeadline) {
            wantShooter = true;
            wantIntake  = false;
        } else {
            wantShooter = false;
            wantIntake  = true;
        }

        // Match old TeleOp behavior: shooter always on, intake follows logic
        shooterOn = true;         // if you want auto behavior instead, use: shooterOn = wantShooter;
        intakeOn  = wantIntake;

        // ===== MANUAL FORCE-REGISTER FOR SPINDEXER (driver 2 bumpers) =====
        boolean lb2 = gamepad2.left_bumper;
        boolean rb2 = gamepad2.right_bumper;

        if (lb2 && !prev2LeftBumper) {
            spindexer.forceIntakeSlotGreen(telemetry);
        }
        if (rb2 && !prev2RightBumper) {
            spindexer.forceIntakeSlotPurple(telemetry);
        }

        prev2LeftBumper = lb2;
        prev2RightBumper = rb2;

        // ===== TELEMETRY =====
        SpindexerSubsystem.Ball[] s = spindexer.getSlots();

        readyForIntake = !spindexer.isEjecting() && !spindexer.isAutoRotating();
        telemetry.addData("Spd intakeSlot", spindexer.getIntakeSlotIndex());
        telemetry.addData("Spd readyForIntake", readyForIntake);
        telemetry.addData("Spd full", spindexer.isFull());
        telemetry.addData("Spd ejecting", spindexer.isEjecting());
        telemetry.addData("Spd slot[0]", s[0]);
        telemetry.addData("Spd slot[1]", s[1]);
        telemetry.addData("Spd slot[2]", s[2]);
        telemetry.addData("Spd angle(enc)", "%.1f", spindexer.getCurrentAngleDeg());
        spindexer.debugAbsAngle(telemetry);

        // shooter telemetry (performance only; PID values are finalized)
        double target = shooter.getTargetRpm();
        double current = shooter.getCurrentRpmEstimate();
        double error = target - current;

        telemetry.addData("Shooter On", shooter.isOn());
        telemetry.addData("Field Pos", (fieldPos == 0) ? "NEAR" : "FAR");
        telemetry.addData("Target RPM", "%.0f", target);
        telemetry.addData("Current RPM (est)", "%.0f", current);
        telemetry.addData("RPM Error", "%.0f", error);

        telemetry.addData("Pattern Tag", driverPatternTag);
        telemetry.addData("Pattern Order", spindexer.getGamePattern());

        // Pedro follower debug
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        // === DASHBOARD PACKET (for graphs) ===
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("rpm", current);
        packet.put("target", target);
        packet.put("error", error);
        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
    }
}
