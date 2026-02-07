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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LoaderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF_dualMotor;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem_Passive_State_new_Incremental;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystemIncremental;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.util.MotorMonitor;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp with pedro and pidf _ incremental (FAST)_blue", group = "Test")
public class TeleOp_pedro_pidf_Incre_blue_new extends OpMode {

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

    // ===== DASHBOARD =====
    private FtcDashboard dashboard;
    private MotorMonitor monitor;

    // ===== REV HUB BULK CACHE =====
    private List<LynxModule> hubs;

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
    // --- spindexer trim button edges ---
    private boolean prevTrimPlus = false;
    private boolean prevTrimMinus = false;


    // turret mode cycle edge
    private boolean prevModeX = false;

    // ===== Shooter manual spin-up toggle (gamepad1.rb) =====
    private boolean shooterManualHold = false;
    private boolean prevRB1 = false;

    // ===== SHOOT POSES (tune these numbers) =====
    public static Pose SHOOT_POSE_NEAR = new Pose(48, 96, Math.toRadians(135));
    public static Pose SHOOT_POSE_FAR  = new Pose(57, 20, Math.toRadians(120));

    public static double AIM_OFFSET_NEAR_DEG = 0.0;
    public static double AIM_OFFSET_FAR_DEG  = -4.0;

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

    // Face point in ODO mode (Pedro coords)
    public static double ODO_FACE_X = 0.0;
    public static double ODO_FACE_Y = 144.0;

    // ===== TELEMETRY MODE SWITCH =====
    // true = your current full telemetry
    // false = minimized telemetry (only the essentials you listed)
    public static boolean FULL_TELEMETRY = false;

    // (optional) also gate MotorMonitor since it can be expensive at high Hz
    public static boolean ENABLE_MOTOR_MONITOR = false;


    // ===== POST-PATH ACTIONS =====
    private boolean postPathActionsDone = false;

    private boolean waitingForTagAfterAssist = false;
    public static int TAG_STABLE_FRAMES = 3;
    private int tagStableCount = 0;

    private long lastPanelsUpdateMs = 0;
    private static final long PANELS_PERIOD_MS = 100; // 10 Hz

    // ===== MOTOR MONITOR THROTTLE =====
    public static long MONITOR_PERIOD_MS = 200; // 5 Hz
    private long lastMonitorMs = 0;

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

    // ===== PROFILING (ms) =====
    private double msLoop, msFollower, msSpindexer, msIntake, msShooter, msTurret, msMonitor;
    private double worstLoop, worstFollower, worstSpindexer, worstIntake, worstShooter, worstTurret, worstMonitor;

    private static double nsToMs(long ns) { return ns / 1e6; }

    @Override
    public void init() {

        // REV bulk caching (FAST)
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

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

        // IMPORTANT: do not gate spindexer I2C behind shooter anymore (this caused your “shoots when turning off” weirdness)
        // Keep sensors running continuously.
        spindexer.setI2cAllowed(true);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        monitor = new MotorMonitor(hardwareMap, telemetry, 10,
                "FrontLeft","BackLeft","FrontRight","BackRight",
                "spindexerMotor","intakeMotor","motor_one","turretMotor")
                .setCurrentRateHz(5)          // current slower than vel/pwr
                .setSingleTelemetryItem(true) // FASTEST
                .setShowVelocity(true)
                .setShowPower(true)
                .setShowCurrent(false);//expensive

        telemetry.addLine("TeleOp FAST: bulk cache + profiling + throttled monitor");
        telemetry.update();
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
        long tLoop0 = System.nanoTime();
        long now = System.currentTimeMillis();

        // ===== BULK CACHE CLEAR (must be first) =====
        for (LynxModule hub : hubs) hub.clearBulkCache();

        // ===== UPDATE PEDRO FOLLOWER =====
        long t = System.nanoTime();
        follower.update();
        msFollower = nsToMs(System.nanoTime() - t);
        worstFollower = Math.max(worstFollower, msFollower);

        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

        // ===== DRIVETRAIN =====
        double leftX  = gamepad2.left_stick_x;
        double leftY  = gamepad2.left_stick_y;
        double rightX = gamepad2.left_stick_x; // <-- if you actually meant right stick, change to gamepad2.right_stick_x
        rightX = gamepad2.right_stick_x;

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

        if (shootAssistActive && (b1Edge || driverOverride)) {
            cancelShootAssist();
        }

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
            double txDeg = vision.getGoalTxDegOrNaN();
            if (!Double.isNaN(txDeg)) tagStableCount++;
            else tagStableCount = 0;

            if (tagStableCount >= TAG_STABLE_FRAMES) {
                waitingForTagAfterAssist = false;
                turretAimMode = TurretAimMode.VISION_TRACK;
                turretPositionCommandActive = true;
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
                    double txDeg = vision.getGoalTxDegOrNaN();
                    double offset = (fieldPos == 0) ? AIM_OFFSET_NEAR_DEG : AIM_OFFSET_FAR_DEG;

                    if (!Double.isNaN(txDeg)) {
                        double aimErrDeg = TX_SIGN * (txDeg + offset);

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

        t = System.nanoTime();
        turret.update();
        msTurret = nsToMs(System.nanoTime() - t);
        worstTurret = Math.max(worstTurret, msTurret);

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

        boolean trimPlus  = gamepad1.dpad_up;
        boolean trimMinus = gamepad1.dpad_down;

        if (trimPlus && !prevTrimPlus)  spindexer.trimPlusStep();
        if (trimMinus && !prevTrimMinus) spindexer.trimMinusStep();

        prevTrimPlus = trimPlus;
        prevTrimMinus = trimMinus;


        // ===== SPINDEXER UPDATE (pass null telemetry to avoid per-loop DS spam) =====
        spindexer.setI2cAllowed(true);

        t = System.nanoTime();
        spindexerIsFull = spindexer.update(null, loader, yEdge, driverPatternTag);
        msSpindexer = nsToMs(System.nanoTime() - t);
        worstSpindexer = Math.max(worstSpindexer, msSpindexer);

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

        // Update loader after spindexer
        loader.updateLoader();

        // Compute hasAnyBall locally
        SpindexerSubsystem_Passive_State_new_Incremental.Ball[] slots = spindexer.getSlots();
        boolean hasAnyBall = false;
        for (SpindexerSubsystem_Passive_State_new_Incremental.Ball b : slots) {
            if (b != SpindexerSubsystem_Passive_State_new_Incremental.Ball.EMPTY) { hasAnyBall = true; break; }
        }

        // ===== Eject / shooter timing =====
        boolean ejecting = spindexer.isEjecting();
        if (lastEjecting && !ejecting && !hasAnyBall) {
            shooterSpinDownDeadline = now + 2000;
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

        if (shooterManualHold) {
            wantShooter = true;
        }

        shooterOn = wantShooter;
        intakeOn  = wantIntake;

        // ===== SHOOTER UPDATE =====
        t = System.nanoTime();
        shooter.update(
                shooterOn,
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                fieldPos
        );
        msShooter = nsToMs(System.nanoTime() - t);
        worstShooter = Math.max(worstShooter, msShooter);

        // ===== INTAKE UPDATE =====
        if (intakeOn) intake.startIntake();
        else intake.stopIntake();

        t = System.nanoTime();
        intake.update();
        msIntake = nsToMs(System.nanoTime() - t);
        worstIntake = Math.max(worstIntake, msIntake);

        // ===== AUTOFIRE PULSE (shoot assist) =====
        if (shootAssistActive) {
            boolean arrived = atPose(follower.getPose(), activeShootPose, 2.0, 6.0);

            double targetRpm  = shooter.getTargetRpm();
            double currentRpm = shooter.getCurrentRpmEstimate();
            boolean rpmReady  = Math.abs(targetRpm - currentRpm) < 150;

            double txDeg = vision.getGoalTxDegOrNaN();
            double offset = (fieldPos == 0) ? AIM_OFFSET_NEAR_DEG : AIM_OFFSET_FAR_DEG;

            boolean visionAimed = !Double.isNaN(txDeg) &&
                    Math.abs(TX_SIGN * (txDeg + offset)) <= TX_DEADBAND_DEG;

            boolean turretReady = !waitingForTagAfterAssist
                    && (turretAimMode == TurretAimMode.VISION_TRACK)
                    && visionAimed;

            if (postPathActionsDone && turretReady && arrived && rpmReady && now > autoFireCooldownUntil) {
                autoFirePulse = true;
                autoFireCooldownUntil = now + 800;
            }
        }

        PoseStorage.lastPose = follower.getPose();

        // ===== THROTTLED MOTOR MONITOR =====
        if (ENABLE_MOTOR_MONITOR) {
            if (now - lastMonitorMs >= MONITOR_PERIOD_MS) {
                t = System.nanoTime();
                monitor.update();
                msMonitor = nsToMs(System.nanoTime() - t);
                worstMonitor = Math.max(worstMonitor, msMonitor);
                lastMonitorMs = now;
            } else {
                msMonitor = 0;
            }
        }

        // ===== PROFILING: loop =====
        msLoop = nsToMs(System.nanoTime() - tLoop0);
        worstLoop = Math.max(worstLoop, msLoop);

        // ===== THROTTLED TELEMETRY UPDATE =====
        long nowMs = System.currentTimeMillis();
        if (nowMs - lastPanelsUpdateMs >= PANELS_PERIOD_MS) {
            lastPanelsUpdateMs = nowMs;

            if (FULL_TELEMETRY) {
                telemetryFull(slots);
            } else {
                telemetryMinimal(slots);
            }



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

    private void telemetryMinimal(SpindexerSubsystem_Passive_State_new_Incremental.Ball[] slots) {
        // -------- MINIMAL DS TELEMETRY --------
        telemetry.addData("Field", (fieldPos == 0) ? "NEAR" : "FAR");
        telemetry.addData("TurretMode", turretAimMode);

        // Shooter
        telemetry.addData("Shooter Target RPM", "%.0f", shooter.getTargetRpm());

        // Spindexer
        telemetry.addData("Spd state", spindexer.getStateName()); // needs the small spindexer addition below
        telemetry.addData("Spd tag", spindexer.getGameTag());
        telemetry.addData("Spd slot0", slots[0]);
        telemetry.addData("Spd slot1", slots[1]);
        telemetry.addData("Spd slot2", slots[2]);
        telemetry.addData("Pattern", patternStringForTag(driverPatternTag));

        // -------- MINIMAL PANELS --------
        telemetryM.addData("Field", (fieldPos == 0) ? 0 : 1);
        telemetryM.addData("TurretMode", turretAimMode.toString());
        telemetryM.addData("Shooter/target_rpm", shooter.getTargetRpm());
        telemetryM.addData("Spd/state", spindexer.getStateName());
        telemetryM.addData("Spd/tag", spindexer.getGameTag());
        telemetryM.addData("Spd/s0", slots[0].toString());
        telemetryM.addData("Spd/s1", slots[1].toString());
        telemetryM.addData("Spd/s2", slots[2].toString());
    }

    private void telemetryFull(SpindexerSubsystem_Passive_State_new_Incremental.Ball[] slots) {
        telemetryM.addData("ms/loop", msLoop);
        telemetryM.addData("ms/follower", msFollower);
        telemetryM.addData("ms/spindexer", msSpindexer);
        telemetryM.addData("ms/intake", msIntake);
        telemetryM.addData("ms/shooter", msShooter);
        telemetryM.addData("ms/turret", msTurret);
        telemetryM.addData("ms/monitor", msMonitor);

        telemetryM.addData("worst/loop", worstLoop);
        telemetryM.addData("worst/follower", worstFollower);
        telemetryM.addData("worst/spindexer", worstSpindexer);
        telemetryM.addData("worst/intake", worstIntake);
        telemetryM.addData("worst/shooter", worstShooter);
        telemetryM.addData("worst/turret", worstTurret);
        telemetryM.addData("worst/monitor", worstMonitor);

        telemetryM.addData("spdx/ms_total", spindexer.getUpdateMs());
        telemetryM.addData("spdx/ms_sense", spindexer.getUpdateMsSense());
        telemetryM.addData("spdx/ms_i2c", spindexer.getUpdateMsHw());
        telemetryM.addData("spdx/ms_motion", spindexer.getUpdateMsMotion());
        telemetryM.addData("spdx/ms_persist", spindexer.getUpdateMsPersist());

        telemetry.addData("Loop ms", "%.2f (worst %.2f)", msLoop, worstLoop);
        telemetry.addData("Spindexer ms", "%.2f (worst %.2f)", msSpindexer, worstSpindexer);
        telemetry.addData("Follower ms", "%.2f (worst %.2f)", msFollower, worstFollower);
        telemetry.addData("Monitor ms", "%.2f (worst %.2f)", msMonitor, worstMonitor);

        telemetry.addData("Spd full", spindexerIsFull);
        telemetry.addData("Spd ejecting", spindexer.isEjecting());
        telemetry.addData("slot0", slots[0]);
        telemetry.addData("slot1", slots[1]);
        telemetry.addData("slot2", slots[2]);
    }
    private static String patternStringForTag(int tag) {
        switch (tag) {
            case 23: return "P-P-G";
            case 22: return "P-G-P";
            case 21: return "G-P-P";
            case 0:  return "Fast (no pattern)";
            default: return "Unknown(tag=" + tag + ")";
        }
    }
}
