package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.util.Range;
import com.bylazar.utils.LoopTimer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF_dualMotor;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_Passive_State_new_Incremental;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystemIncremental;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.subsystems.util.BulkCacheManager;
import org.firstinspires.ftc.teamcode.subsystems.util.MotorMonitor;

@Configurable
@TeleOp(name = "TeleOp with pedro and pidf _ incremental", group = "Test")
public class TeleOp_pedro_pidf_Incre extends OpMode {

    // ===== PEDRO FOLLOWER / DRIVE =====
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;

    // ===== SUBSYSTEMS =====
    private ShooterSubsystemFF_dualMotor shooter;
    private IntakeSubsystem intake;
    private SpindexerSubsystem_Passive_State_new_Incremental spindexer;
    private LoaderSubsystem loader;
    private TurretSubsystemIncremental turret;
    private VisionSubsystem vision;
    private BulkCacheManager bulk;

    // ===== DASHBOARD =====
    private FtcDashboard dashboard;
    private MotorMonitor monitor;

    // ===== STATE VARIABLES =====
    private boolean prev2a = false;

    private int fieldPos = 0; // 0 near, 1 far
    private boolean spindexerIsFull = false;

    private boolean shooterOn = false;
    private boolean intakeOn = false;

    // turret preset edges
    private boolean prevA = false, prevDpadL = false, prevDpadR = false;

    private long shooterSpinDownDeadline = 0;
    private boolean lastEjecting = false;

    // driver2 pattern select
    private int driverPatternTag = -1;
    private boolean prev2Up = false, prev2Right = false, prev2Left = false, prev2Down = false;

    // edges
    private boolean prevLeftStick = false;
    private boolean prevRehome = false;
    private boolean prevY = false;
    private boolean prevB1 = false;

    // turret mode cycle edge
    private boolean prevModeX = false;

    // ===== Shooter manual spin-up toggle (gamepad1.rb) =====
    private boolean shooterManualHold = false;
    private boolean prevRB1 = false;

    // ===== SHOOT POSES (tune these numbers) =====
    public static Pose SHOOT_POSE_NEAR = new Pose(80, 80, Math.toRadians(45));
    public static Pose SHOOT_POSE_FAR  = new Pose(80.6, 17, Math.toRadians(90));

    public static double AIM_OFFSET_NEAR_DEG = 0.0;
    public static double AIM_OFFSET_FAR_DEG  = 4.0;

    // ===== SHOOT ASSIST STATE =====
    private boolean shootAssistActive = false;
    private boolean prevShootAssistBtn = false;
    private Pose activeShootPose = SHOOT_POSE_NEAR;

    private boolean autoFirePulse = false;
    private long autoFireCooldownUntil = 0;

    // ===== TURRET OWNER STATE =====
    private boolean turretPositionCommandActive = false;

    // ===== Vision->Turret tracking =====
    public static double TX_DEADBAND_DEG = 0.5;
    public static double TX_MAX_STEP_DEG = 2.0;
    public static double TX_SIGN = 1.0;
    public static double TURRET_ANGLE_TOL_DEG = 3.0;

    private enum TurretAimMode { MANUAL_HOLD, VISION_TRACK, ODO_FACE_POINT }
    private TurretAimMode turretAimMode = TurretAimMode.MANUAL_HOLD;

    private TurretAimMode turretAimModeBeforeAssist = TurretAimMode.MANUAL_HOLD;

    private boolean assistVisionEnabled = false;

    public static double ODO_FACE_X = 132.0;
    public static double ODO_FACE_Y = 136.0;

    private boolean postPathActionsDone = false;

    private boolean waitingForTagAfterAssist = false;
    public static int TAG_STABLE_FRAMES = 3;
    private int tagStableCount = 0;

    // ===== LOOP TIMER / PANELS RATE LIMIT =====
    private final LoopTimer loopTimer = new LoopTimer(10);
    private boolean loopTimerPrimed = false;
    private long worstMs = 0;
    private long lastPanelsUpdateMs = 0;
    private static final long PANELS_PERIOD_MS = 100; // 10 Hz

    // ===== RUMBLE WHEN SPINDEXER FULL =====
    public static boolean RUMBLE_WHEN_FULL = true;
    public static int FULL_RUMBLE_MS = 160;
    public static double FULL_RUMBLE_POWER = 0.6;

    public static boolean FULL_TREMBLE_WHILE_FULL = true;
    public static long FULL_TREMBLE_PERIOD_MS = 700;
    public static int  FULL_TREMBLE_MS = 90;
    public static double FULL_TREMBLE_POWER = 0.25;

    private boolean prevSpindexerFull = false;
    private long lastFullTrembleMs = 0;

    // =========================
    // ===== SUBSYSTEM PROFILING
    // =========================
    private static double nsToMs(long ns) { return ns / 1e6; }

    private double msFollower=0, msDrawing=0, msTurret=0, msSpindexer=0, msLoader=0, msShooter=0, msIntake=0, msMonitor=0;
    private double worstFollower=0, worstDrawing=0, worstTurret=0, worstSpindexer=0, worstLoader=0, worstShooter=0, worstIntake=0, worstMonitor=0;
    private double msSum=0, worstSum=0;

    @Override
    public void init() {
        // initailize bulk reading
        bulk = new BulkCacheManager(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        Drawing.init();

        Pose startPose =
                (PoseStorage.lastPose != null) ? PoseStorage.lastPose :
                        (startingPose != null) ? startingPose :
                                Drawing.getStartingPose();

        follower.setStartingPose(startPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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

        shooter   = new ShooterSubsystemFF_dualMotor(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        intake    = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem_Passive_State_new_Incremental(hardwareMap);
        turret    = new TurretSubsystemIncremental(hardwareMap);
        vision    = new VisionSubsystem(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        monitor = new MotorMonitor(
                hardwareMap, telemetry, 50,
                "FrontLeft","BackLeft","FrontRight","BackRight","spindexerMotor","intakeMotor","motor_one","turretMotor"
        );

        telemetry.addLine("TeleOp with Pedro + Shooter PIDF (Incremental)");
        telemetry.update();

        loopTimer.start();
        loopTimerPrimed = true;
        worstMs = 0;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        shooterSpinDownDeadline = 0;

        turret.goToAngle(turret.getCurrentAngleDeg());
        turretPositionCommandActive = true;
    }

    @Override
    public void loop() {
        long nowMsWall = System.currentTimeMillis();

        // =========================
        // ===== LOOP TIME (ms/hz)
        // =========================
        long loopMs = 0;
        double loopHz = 0;

        if (loopTimerPrimed) {
            loopTimer.end();
            loopMs = loopTimer.getMs();
            loopHz = loopTimer.getHz();
            if (loopMs > worstMs) worstMs = loopMs;
        }
        loopTimer.start();

        // Optional: if your BulkCacheManager has a "clear/refresh" call, do it here and profile it.
        // (I’m not guessing method names to avoid compile errors.)

        // =========================
        // ===== FOLLOWER UPDATE
        // =========================
        long t0 = System.nanoTime();
        follower.update();
        msFollower = nsToMs(System.nanoTime() - t0);
        if (msFollower > worstFollower) worstFollower = msFollower;

        // =========================
        // ===== DRAWING (can be non-trivial)
        // =========================
        t0 = System.nanoTime();
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
        msDrawing = nsToMs(System.nanoTime() - t0);
        if (msDrawing > worstDrawing) worstDrawing = msDrawing;

        // ===== DRIVETRAIN =====
        double leftX  = gamepad2.left_stick_x;
        double leftY  = gamepad2.left_stick_y;
        double rightX = gamepad2.right_stick_x;

        boolean a2 = gamepad2.a;
        if (a2 && !prev2a) slowMode = !slowMode;
        prev2a = a2;

        double driveScale = slowMode ? 0.4 : 1.0;

        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -leftY * driveScale,
                    -leftX * driveScale,
                    -rightX * driveScale,
                    true
            );
        }

        // ===== SHOOT ASSIST TOGGLE (gamepad2.y) =====
        boolean shootAssistBtn = gamepad2.y;
        boolean shootAssistEdge = shootAssistBtn && !prevShootAssistBtn;
        prevShootAssistBtn = shootAssistBtn;

        boolean b1Edge = gamepad1.b && !prevB1;
        prevB1 = gamepad1.b;

        boolean driverOverride =
                Math.abs(gamepad2.left_stick_x) > 0.2 ||
                        Math.abs(gamepad2.left_stick_y) > 0.2 ||
                        Math.abs(gamepad2.right_stick_x) > 0.2;

        if (shootAssistEdge) {
            if (!shootAssistActive) {
                Pose target = (fieldPos == 0) ? SHOOT_POSE_NEAR : SHOOT_POSE_FAR;
                startShootAssist(target);
            } else {
                cancelShootAssist();
            }
        }

        if (shootAssistActive && (b1Edge || driverOverride)) cancelShootAssist();

        if (shootAssistActive && automatedDrive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;

            if (!postPathActionsDone) {
                turretAimMode = TurretAimMode.MANUAL_HOLD;
                turret.goToAngle(0.0);
                turretPositionCommandActive = true;

                waitingForTagAfterAssist = true;
                tagStableCount = 0;
                postPathActionsDone = true;
            }
        }

        if (shootAssistActive && postPathActionsDone && waitingForTagAfterAssist) {
            double tx = vision.getGoalTxDegOrNaN();
            if (!Double.isNaN(tx)) tagStableCount++;
            else tagStableCount = 0;

            if (tagStableCount >= TAG_STABLE_FRAMES) {
                waitingForTagAfterAssist = false;
                turretAimMode = TurretAimMode.VISION_TRACK;
                turretPositionCommandActive = true;
                assistVisionEnabled = true;
            } else {
                turretAimMode = TurretAimMode.MANUAL_HOLD;
                turret.goToAngle(0.0);
                turretPositionCommandActive = true;
            }
        }

        // ===== Y edge (user OR auto-fire pulse) =====
        boolean yUser = gamepad1.y;
        boolean yEdgeUser = yUser && !prevY;
        prevY = yUser;

        boolean yEdge = yEdgeUser || autoFirePulse;
        autoFirePulse = false;

        // ===== FIELD POS TOGGLE (gamepad1 left stick button) =====
        boolean leftStickButton = gamepad1.left_stick_button;
        if (leftStickButton && !prevLeftStick) fieldPos = (fieldPos == 0) ? 1 : 0;
        prevLeftStick = leftStickButton;

        // ===== TURRET MODE CYCLE (gamepad1.x) =====
        boolean modeX = gamepad1.x;
        boolean modeEdge = modeX && !prevModeX;
        prevModeX = modeX;

        if (modeEdge) {
            switch (turretAimMode) {
                case MANUAL_HOLD:    turretAimMode = TurretAimMode.VISION_TRACK; break;
                case VISION_TRACK:   turretAimMode = TurretAimMode.ODO_FACE_POINT; break;
                case ODO_FACE_POINT: turretAimMode = TurretAimMode.MANUAL_HOLD; break;
            }
        }

        // ===== PRESET ANGLES =====
        boolean a  = gamepad1.a;
        boolean dl = gamepad1.dpad_left;
        boolean dr = gamepad1.dpad_right;

        boolean aEdge  = a  && !prevA;
        boolean dlEdge = dl && !prevDpadL;
        boolean drEdge = dr && !prevDpadR;

        prevA = a;
        prevDpadL = dl;
        prevDpadR = dr;

        if (aEdge)  { turretAimMode = TurretAimMode.MANUAL_HOLD; turret.goToAngle(0.0);    turretPositionCommandActive = true; }
        if (dlEdge) { turretAimMode = TurretAimMode.MANUAL_HOLD; turret.goToAngle(-90.0); turretPositionCommandActive = true; }
        if (drEdge) { turretAimMode = TurretAimMode.MANUAL_HOLD; turret.goToAngle(90.0);  turretPositionCommandActive = true; }

        // ===== TURRET MANUAL/TRACKING =====
        double stickX = gamepad1.right_stick_x;
        boolean manualActive = Math.abs(stickX) > 0.05;
        if (manualActive) waitingForTagAfterAssist = false;

        if (manualActive) {
            turretAimMode = TurretAimMode.MANUAL_HOLD;
            turretPositionCommandActive = false;
            turret.setManualPower(stickX * 0.5);
        } else {
            switch (turretAimMode) {
                case VISION_TRACK: {
                    double tx = vision.getGoalTxDegOrNaN();
                    double offset = (fieldPos == 0) ? AIM_OFFSET_NEAR_DEG : AIM_OFFSET_FAR_DEG;

                    if (!Double.isNaN(tx)) {
                        double aimErrDeg = TX_SIGN * (tx + offset);
                        if (Math.abs(aimErrDeg) > TX_DEADBAND_DEG) {
                            double step = Range.clip(aimErrDeg, -TX_MAX_STEP_DEG, TX_MAX_STEP_DEG);
                            turret.goToAngle(turret.getCurrentAngleDeg() + step);
                            turretPositionCommandActive = true;
                        } else {
                            turret.goToAngle(turret.getCurrentAngleDeg());
                            turretPositionCommandActive = true;
                        }
                    } else {
                        turret.goToAngle(turret.getCurrentAngleDeg());
                        turretPositionCommandActive = true;
                    }
                    break;
                }

                case ODO_FACE_POINT: {
                    turret.faceTarget(ODO_FACE_X, ODO_FACE_Y, follower.getPose());
                    turretPositionCommandActive = true;
                    break;
                }

                case MANUAL_HOLD:
                default: {
                    if (turretPositionCommandActive) {
                        double err = turret.getTargetAngleDeg() - turret.getCurrentAngleDeg();
                        if (Math.abs(err) < TURRET_ANGLE_TOL_DEG) turretPositionCommandActive = false;
                    } else {
                        turret.setManualPower(0.0);
                    }
                    break;
                }
            }
        }

        // ===== PROFILE: turret.update() =====
        t0 = System.nanoTime();
        turret.update();
        msTurret = nsToMs(System.nanoTime() - t0);
        if (msTurret > worstTurret) worstTurret = msTurret;

        // ===== SPINDEXER COMMAND (GO TO INTAKE, NOT rezero) =====
        boolean rehomeButton = gamepad1.right_stick_button;
        if (rehomeButton && !prevRehome) spindexer.homeToIntake();
        prevRehome = rehomeButton;

        // manual shooter spinup
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) shooterManualHold = !shooterManualHold;
        prevRB1 = rb1;

        // ===== DRIVER2 PATTERN SELECT =====
        boolean dUp2    = gamepad2.dpad_up;
        boolean dRight2 = gamepad2.dpad_right;
        boolean dLeft2  = gamepad2.dpad_left;
        boolean dDown2  = gamepad2.dpad_down;

        if (dUp2 && !prev2Up) driverPatternTag = 23;
        if (dRight2 && !prev2Right) driverPatternTag = 22;
        if (dLeft2 && !prev2Left) driverPatternTag = 21;
        if (dDown2 && !prev2Down) driverPatternTag = 0;

        prev2Up = dUp2;
        prev2Right = dRight2;
        prev2Left = dLeft2;
        prev2Down = dDown2;

        // ===== PROFILE: spindexer.update() =====
        spindexer.setI2cAllowed(!shooter.isOn());
        t0 = System.nanoTime();
        spindexerIsFull = spindexer.update(telemetry, loader, yEdge, driverPatternTag);
        msSpindexer = nsToMs(System.nanoTime() - t0);
        if (msSpindexer > worstSpindexer) worstSpindexer = msSpindexer;

        // ===== GAMEPAD RUMBLE WHEN SPINDEXER FULL =====
        if (RUMBLE_WHEN_FULL) {
            boolean fullNow = spindexerIsFull;

            if (fullNow && !prevSpindexerFull) {
                gamepad2.rumble(FULL_RUMBLE_POWER, FULL_RUMBLE_POWER, FULL_RUMBLE_MS);
                lastFullTrembleMs = System.currentTimeMillis();
            }

            if (FULL_TREMBLE_WHILE_FULL && fullNow) {
                long nowMs = System.currentTimeMillis();
                if (nowMs - lastFullTrembleMs >= FULL_TREMBLE_PERIOD_MS) {
                    gamepad2.rumble(FULL_TREMBLE_POWER, FULL_TREMBLE_POWER, FULL_TREMBLE_MS);
                    lastFullTrembleMs = nowMs;
                }
            }

            prevSpindexerFull = fullNow;
        }

        // ===== PROFILE: loader.updateLoader() =====
        t0 = System.nanoTime();
        loader.updateLoader();
        msLoader = nsToMs(System.nanoTime() - t0);
        if (msLoader > worstLoader) worstLoader = msLoader;

        // Compute hasAnyBall locally from slots
        SpindexerSubsystem_Passive_State_new_Incremental.Ball[] slots = spindexer.getSlots();
        boolean hasAnyBall = false;
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] != SpindexerSubsystem_Passive_State_new_Incremental.Ball.EMPTY) {
                hasAnyBall = true;
                break;
            }
        }

        // ===== Eject / shooter timing =====
        boolean ejecting = spindexer.isEjecting();
        if (lastEjecting && !ejecting && !hasAnyBall) {
            shooterSpinDownDeadline = nowMsWall + 2000;
        }
        lastEjecting = ejecting;

        boolean wantShooter;
        boolean wantIntake;

        if (spindexerIsFull) {
            wantShooter = true;
            wantIntake  = false;
        } else if (ejecting || nowMsWall < shooterSpinDownDeadline) {
            wantShooter = true;
            wantIntake  = false;
        } else {
            wantShooter = false;
            wantIntake  = true;
        }

        if (shooterManualHold) wantShooter = true;

        shooterOn = wantShooter;
        intakeOn  = wantIntake;

        // ===== PROFILE: shooter.update() =====
        t0 = System.nanoTime();
        shooter.update(
                shooterOn,
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                fieldPos
        );
        msShooter = nsToMs(System.nanoTime() - t0);
        if (msShooter > worstShooter) worstShooter = msShooter;

        // ===== PROFILE: intake update =====
        t0 = System.nanoTime();
        if (intakeOn) intake.startIntake();
        else intake.stopIntake();
        intake.update();
        msIntake = nsToMs(System.nanoTime() - t0);
        if (msIntake > worstIntake) worstIntake = msIntake;

        // ===== AUTOFIRE PULSE (shoot assist) =====
        if (shootAssistActive) {
            boolean arrived = atPose(follower.getPose(), activeShootPose, 2.0, 6.0);

            double targetRpm  = shooter.getTargetRpm();
            double currentRpm = shooter.getCurrentRpmEstimate();
            boolean rpmReady  = Math.abs(targetRpm - currentRpm) < 150;

            double tx = vision.getGoalTxDegOrNaN();
            double offset = (fieldPos == 0) ? AIM_OFFSET_NEAR_DEG : AIM_OFFSET_FAR_DEG;

            boolean visionAimed = !Double.isNaN(tx) &&
                    Math.abs(TX_SIGN * (tx + offset)) <= TX_DEADBAND_DEG;

            boolean turretReady = !waitingForTagAfterAssist
                    && (turretAimMode == TurretAimMode.VISION_TRACK)
                    && visionAimed;

            if (postPathActionsDone && turretReady && arrived && rpmReady && nowMsWall > autoFireCooldownUntil) {
                autoFirePulse = true;
                autoFireCooldownUntil = nowMsWall + 800;
            }
        }

        PoseStorage.lastPose = follower.getPose();

        // ===== PROFILE: monitor.update() =====
        t0 = System.nanoTime();
        monitor.update();
        msMonitor = nsToMs(System.nanoTime() - t0);
        if (msMonitor > worstMonitor) worstMonitor = msMonitor;

        // ===== SUM (your “where is the time going?” number) =====
        msSum = msFollower + msDrawing + msTurret + msSpindexer + msLoader + msShooter + msIntake + msMonitor;
        if (msSum > worstSum) worstSum = msSum;

        // ===== TELEMETRY / PANELS (10 Hz) =====
        if (nowMsWall - lastPanelsUpdateMs >= PANELS_PERIOD_MS) {
            lastPanelsUpdateMs = nowMsWall;

            // Loop stats
            telemetryM.addData("loop/ms", loopMs);
            telemetryM.addData("loop/hz", loopHz);
            telemetryM.addData("loop/worst_ms", worstMs);

            // Subsystem profiling
            telemetryM.addData("prof/sum_ms", msSum);
            telemetryM.addData("prof/sum_worst_ms", worstSum);

            telemetryM.addData("prof/follower_ms", msFollower);
            telemetryM.addData("prof/drawing_ms", msDrawing);
            telemetryM.addData("prof/turret_ms", msTurret);
            telemetryM.addData("prof/spindexer_ms", msSpindexer);
            telemetryM.addData("prof/loader_ms", msLoader);
            telemetryM.addData("prof/shooter_ms", msShooter);
            telemetryM.addData("prof/intake_ms", msIntake);
            telemetryM.addData("prof/monitor_ms", msMonitor);

            telemetryM.addData("prof/follower_worst", worstFollower);
            telemetryM.addData("prof/drawing_worst", worstDrawing);
            telemetryM.addData("prof/turret_worst", worstTurret);
            telemetryM.addData("prof/spindexer_worst", worstSpindexer);
            telemetryM.addData("prof/loader_worst", worstLoader);
            telemetryM.addData("prof/shooter_worst", worstShooter);
            telemetryM.addData("prof/intake_worst", worstIntake);
            telemetryM.addData("prof/monitor_worst", worstMonitor);

            // Keep some of your existing telemetry (minimal so telemetry doesn’t dominate)
            telemetry.addData("Loop ms", loopMs);
            telemetry.addData("Worst ms", worstMs);
            telemetry.addData("Sum ms", "%.2f", msSum);
            telemetry.addData("Shooter ms", "%.2f", msShooter);
            telemetry.addData("Spindexer ms", "%.2f", msSpindexer);

            telemetryM.update(telemetry);
            telemetry.update();
        }
    }

    private PathChain buildLineToPose(Pose target) {
        return follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, target)))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                target.getHeading(),
                                0.8
                        )
                )
                .build();
    }

    private void startShootAssist(Pose target) {
        activeShootPose = target;
        turretAimModeBeforeAssist = turretAimMode;

        turretAimMode = TurretAimMode.MANUAL_HOLD;
        turretPositionCommandActive = false;
        assistVisionEnabled = false;

        follower.followPath(buildLineToPose(target));
        automatedDrive = true;
        shootAssistActive = true;
        postPathActionsDone = false;
    }

    private void cancelShootAssist() {
        follower.startTeleopDrive();
        automatedDrive = false;
        shootAssistActive = false;
        postPathActionsDone = false;

        waitingForTagAfterAssist = false;
        tagStableCount = 0;

        turretAimMode = turretAimModeBeforeAssist;
        assistVisionEnabled = false;
    }

    private static double wrapRad(double r) {
        while (r > Math.PI) r -= 2.0 * Math.PI;
        while (r < -Math.PI) r += 2.0 * Math.PI;
        return r;
    }

    private boolean atPose(Pose cur, Pose target, double posTolIn, double headingTolDeg) {
        double dx = cur.getX() - target.getX();
        double dy = cur.getY() - target.getY();
        double dist = Math.hypot(dx, dy);
        double dh = Math.toDegrees(Math.abs(wrapRad(cur.getHeading() - target.getHeading())));
        return dist <= posTolIn && dh <= headingTolDeg;
    }
}
