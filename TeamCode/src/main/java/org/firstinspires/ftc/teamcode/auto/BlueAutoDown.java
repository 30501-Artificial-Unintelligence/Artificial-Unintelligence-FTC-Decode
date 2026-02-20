package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Configurable
@Autonomous(name = "BlueAutoDown", group = "Auto")
public class BlueAutoDown extends OpMode {

    // =========================
    // ===== POSES / PATHS =====
    // =========================
    private final Pose startPose   = new Pose(57.65, 9.29, Math.toRadians(180));
    private final Pose scorePose   = new Pose(58.9, 18.8, Math.toRadians(180));

    private final Pose prePickup1Pose = new Pose(52, 35, Math.toRadians(180));
    private final Pose pickup1Pose    = new Pose(14, 35, Math.toRadians(180));

    private final Pose prePickup2Pose = new Pose(30, 12, Math.toRadians(180));
    private final Pose pickup2Pose    = new Pose(9, 12, Math.toRadians(180));

    private final Pose prePickup3Pose = new Pose(52, 35, Math.toRadians(180));
    private final Pose pickup3Pose    = new Pose(14, 35, Math.toRadians(180));

    private final Pose parkPose = new Pose(36, 50, Math.toRadians(180));

    private Path scorePreload;
    private PathChain goPrePickup1, creepToPickup1, scorePickup1;
    private PathChain goPrePickup2, creepToPickup2, scorePickup2;
    private PathChain goPrePickup3, creepToPickup3, scorePickup3;
    private PathChain goParkFromScore;

    // Power in path
    public static double PWR_FAST  = 0.85;
    public static double PWR_CREEP = 0.30;

    // =========================
    // ===== TURRET / AIM ======
    // =========================

    // PREAIM is ONLY for reading pattern tag (21/22/23)
    public static double PATTERN_TAG_X = 72.0;
    public static double PATTERN_TAG_Y = 160.0;

    // If vision tracking is disabled, face this point (should be same as goal)
    public static double ODO_FACE_X = 8.0;
    public static double ODO_FACE_Y = 128.0;

    // Vision-based turret tracking toggle
    public static boolean VISION_TURRET_TRACKING_ENABLED = true; // if false, odo tracking

    // Which tag ID to track for aiming (goal tag)
    public static int TRACK_TAG_ID = 20;

    // TeleOp-like tracking behavior
    public static double TRACK_TX_DEADBAND_DEG = 0.4;
    public static double TRACK_MAX_STEP_DEG    = 2.0;
    public static double TRACK_SIGN            = 1.0;

    // If tag disappears during tracking, "spin back" to zero after timeout
    public static boolean RETURN_TO_ZERO_ON_TAG_LOST = true;
    public static long TAG_LOST_TIMEOUT_MS = 250;

    // Aim-ready thresholds (used to decide when to fire)
    public static double AIM_TX_READY_DEG   = 0.7;  // when |tx| < this, consider aimed
    public static double AIM_ANGLE_TOL_DEG  = 3.0;  // used in ODO mode

    // turret zero tolerance
    public static double TURRET_ZERO_TOL_DEG = 3.0;

    // how close to scorePose counts as "arrived"
    public static double SCOREPOSE_POS_TOL_IN = 2.0;
    public static double SCOREPOSE_HEAD_TOL_DEG = 4.0;

    public static double PREAIM_MAX_SEC = 1.2; // safety so preaim never deadlocks

    private enum TurretPhase {
        PREAIM_PATTERN,
        HOMING_TO_ZERO,
        VISION_TRACKING,
        ODO_FACE_POINT
    }
    private TurretPhase turretPhase = TurretPhase.PREAIM_PATTERN;

    private boolean turretZeroLatched = false;
    private long lastAimTagSeenMs = 0;

    // --- Fail-safes so shots can’t deadlock ---
    public static long TURRET_HOME_TIMEOUT_MS = 900;
    public static long MAX_AIM_WAIT_MS        = 1400;

    private long turretHomeStartMs = 0;

    // =========================
    // ===== SHOOT SEQUENCE =====
    // =========================
    private boolean shooting = false;
    private boolean yEdgeSent = false;
    private long shootStartMs = 0;

    public static long MIN_SPINUP_MS = 600;   // (optional legacy)
    public static double READY_ERR_RPM = 200; // (optional legacy)

    private boolean ejectEverStarted = false;

    // pattern tag latched from vision (21/22/23) once per match
    private int autoPatternTag = 0; // 0 = no pattern
    private int fieldPos = 1;       // keep your FAR=1 behavior
    public static long SHOOT_SEQUENCE_TIMEOUT_MS = 9000;
    public static int MAX_REEJECT_ATTEMPTS = 2;

    private long shootDeadlineMs = 0;
    private int reEjectAttempts = 0;

    // ===== READY-TO-SHOOT GATING =====
    public static double READY_TOL_RPM = 100.0;
    public static long   READY_STABLE_MS = 150;
    private long readySinceMs = 0;

    // =========================
    // ===== PEDRO / STATE =====
    // =========================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // =========================
    // ===== SUBSYSTEMS =========
    // =========================
    private TurretSubsystemIncremental turret;
    private VisionSubsystem vision;
    private ShooterSubsystemFF_dualMotor shooter;
    private LoaderSubsystem loader;
    private SpindexerSubsystem_Passive_State_new_Incremental spindexer;
    private IntakeSubsystem intake;

    // =========================
    // ===== PATH BUILDING ======
    // =========================
    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        goPrePickup1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, prePickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup1Pose.getHeading())
                .build();

        creepToPickup1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(prePickup1Pose, pickup1Pose)))
                .setLinearHeadingInterpolation(prePickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup1Pose, scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        goPrePickup2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, prePickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup2Pose.getHeading())
                .build();

        creepToPickup2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(prePickup2Pose, pickup2Pose)))
                .setLinearHeadingInterpolation(prePickup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup2Pose, scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        goPrePickup3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, prePickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup3Pose.getHeading())
                .build();

        creepToPickup3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(prePickup3Pose, pickup3Pose)))
                .setLinearHeadingInterpolation(prePickup3Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup3Pose, scorePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading())
                .build();

        goParkFromScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    // =========================
    // ===== PATH STATE MACHINE ==
    // =========================
    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                setPathState(10);
                break;

            case 10:
                if (!follower.isBusy()) {
                    startShootSequence();
                    setPathState(100);
                }
                break;

            case 100:
                if (updateShootSequence(fieldPos, autoPatternTag)) {
                    follower.followPath(goPrePickup1, PWR_FAST, false);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(creepToPickup1, PWR_CREEP, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    startShootSequence();
                    setPathState(101);
                }
                break;

            case 101:
                if (updateShootSequence(fieldPos, autoPatternTag)) {
                    follower.followPath(goPrePickup2, PWR_FAST, false);
                    setPathState(21);
                }
                break;

            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(creepToPickup2, PWR_CREEP, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    startShootSequence();
                    setPathState(999); // you were parking here; keep it
                }
                break;

            case 102:
                if (updateShootSequence(fieldPos, autoPatternTag)) {
                    follower.followPath(goPrePickup3, PWR_FAST, false);
                    setPathState(31);
                }
                break;

            case 31:
                if (!follower.isBusy()) {
                    follower.followPath(creepToPickup3, PWR_CREEP, true);
                    setPathState(32);
                }
                break;

            case 32:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(33);
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    startShootSequence();
                    setPathState(103);
                }
                break;

            case 103:
                if (updateShootSequence(fieldPos, autoPatternTag)) {
                    setPathState(-1);
                }
                break;

            case 999:
                if (updateShootSequence(fieldPos, autoPatternTag)) {
                    follower.followPath(goParkFromScore, PWR_FAST, false);
                    setPathState(-1);
                }
                break;

            case -1:
            default:
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // =========================
    // ===== PATTERN TAG LATCH ==
    // =========================
    private void updateAutoPatternTag() {
        if (autoPatternTag != 0) return;
        int t = vision.getPatternTag();
        if (t == 21 || t == 22 || t == 23) autoPatternTag = t;
    }

    // =========================
    // ===== TURRET CONTROL =====
    // =========================
    private void updateTurretControl() {
        Pose robotPose = follower.getPose();
        long now = System.currentTimeMillis();

        boolean reachedShootPose = atPose(robotPose, scorePose, SCOREPOSE_POS_TOL_IN, SCOREPOSE_HEAD_TOL_DEG);

        int ptag = vision.getPatternTag();
        boolean patternSeen = (ptag == 21 || ptag == 22 || ptag == 23);

        double tx = vision.getTagTxDegOrNaN(TRACK_TAG_ID);
        boolean txValid = !Double.isNaN(tx);
        if (txValid) lastAimTagSeenMs = now;

        switch (turretPhase) {

            case PREAIM_PATTERN: {
                turret.faceTarget(PATTERN_TAG_X, PATTERN_TAG_Y, robotPose);

                boolean preaimTimedOut = pathTimer.getElapsedTimeSeconds() > PREAIM_MAX_SEC;

                if (reachedShootPose || patternSeen || autoPatternTag != 0 || preaimTimedOut) {
                    turretPhase = TurretPhase.HOMING_TO_ZERO;
                    turretZeroLatched = false;
                    turretHomeStartMs = now;
                }
                break;
            }

            case HOMING_TO_ZERO: {
                turret.goToAngle(0.0);

                // fail-safe home latch
                boolean homeTimedOut = (now - turretHomeStartMs) > TURRET_HOME_TIMEOUT_MS;

                if (!turretZeroLatched && (turretAtZero() || homeTimedOut)) {
                    turretZeroLatched = true;
                    turretPhase = VISION_TURRET_TRACKING_ENABLED
                            ? TurretPhase.VISION_TRACKING
                            : TurretPhase.ODO_FACE_POINT;
                }
                break;
            }

            case VISION_TRACKING: {
                if (!VISION_TURRET_TRACKING_ENABLED) {
                    turretPhase = TurretPhase.ODO_FACE_POINT;
                    break;
                }

                if (!txValid) {
                    if (RETURN_TO_ZERO_ON_TAG_LOST && (now - lastAimTagSeenMs) > TAG_LOST_TIMEOUT_MS) {
                        turretPhase = TurretPhase.HOMING_TO_ZERO;
                        turretZeroLatched = false;
                        turretHomeStartMs = now;
                        turret.goToAngle(0.0);
                    } else {
                        turret.goToAngle(turret.getCurrentAngleDeg()); // hold briefly
                    }
                    break;
                }

                if (Math.abs(tx) < TRACK_TX_DEADBAND_DEG) {
                    turret.goToAngle(turret.getCurrentAngleDeg());
                } else {
                    double step = Range.clip(TRACK_SIGN * tx, -TRACK_MAX_STEP_DEG, TRACK_MAX_STEP_DEG);
                    turret.goToAngle(turret.getCurrentAngleDeg() + step);
                }
                break;
            }

            case ODO_FACE_POINT: {
                turret.faceTarget(ODO_FACE_X, ODO_FACE_Y, robotPose);
                break;
            }
        }
    }

    private boolean turretAtZero() {
        return Math.abs(turret.getCurrentAngleDeg()) <= TURRET_ZERO_TOL_DEG;
    }

    private boolean turretAtTarget(double tolDeg) {
        return Math.abs(turret.getTargetAngleDeg() - turret.getCurrentAngleDeg()) <= tolDeg;
    }

    // =========================
    // ===== SHOOT SEQUENCE =====
    // =========================
    private void startShootSequence() {
        shooting = true;
        yEdgeSent = false;
        shootStartMs = System.currentTimeMillis();
        ejectEverStarted = false;

        readySinceMs = 0;

        turretPhase = TurretPhase.HOMING_TO_ZERO;
        turretZeroLatched = false;
        turretHomeStartMs = shootStartMs;

        shootDeadlineMs = shootStartMs + SHOOT_SEQUENCE_TIMEOUT_MS;
        reEjectAttempts = 0;
    }

    private boolean shooterAtSpeedStable() {
        double target = shooter.getTargetRpm();
        double cur = shooter.getCurrentRpmEstimate();
        boolean within = Math.abs(target - cur) <= READY_TOL_RPM;

        long now = System.currentTimeMillis();
        if (within) {
            if (readySinceMs == 0) readySinceMs = now;
            return (now - readySinceMs) >= READY_STABLE_MS;
        } else {
            readySinceMs = 0;
            return false;
        }
    }

    private boolean updateShootSequence(int fieldPos, int tagOverride) {
        if (!shooting) return true;

        long now = System.currentTimeMillis();
        if (now > shootDeadlineMs) {
            shooting = false;
            return true;
        }

        loader.updateLoader();

        updateTurretControl();
        turret.update();

        boolean aimReady;
        if (VISION_TURRET_TRACKING_ENABLED) {
            double tx = vision.getTagTxDegOrNaN(TRACK_TAG_ID);
            boolean txValid = !Double.isNaN(tx);
            aimReady = turretZeroLatched && (!txValid || Math.abs(tx) <= AIM_TX_READY_DEG);
        } else {
            aimReady = turretAtTarget(AIM_ANGLE_TOL_DEG);
        }

        boolean forceAimTimeout = (now - shootStartMs) > MAX_AIM_WAIT_MS;
        boolean allowFire = shooterAtSpeedStable() && (aimReady || forceAimTimeout);

        boolean yEdge = false;
        if (allowFire && !yEdgeSent) {
            yEdge = true;     // exactly one loop
            yEdgeSent = true;
        }

        // PASSIVE spindexer signature
        spindexer.update(telemetry, loader, yEdge, tagOverride);

        if (spindexer.isEjecting()) ejectEverStarted = true;

        if (yEdgeSent && ejectEverStarted && !spindexer.isEjecting()) {

            if (spindexer.hasAnyBall() && reEjectAttempts < MAX_REEJECT_ATTEMPTS) {
                reEjectAttempts++;
                yEdgeSent = false;
                ejectEverStarted = false;
                shootStartMs = now;
                shootDeadlineMs = now + SHOOT_SEQUENCE_TIMEOUT_MS;
                return false;
            }

            shooting = false;
            return true;
        }

        return false;
    }

    // =========================
    // ===== OPMODE LIFECYCLE ===
    // =========================
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        // Use your Drawing helper (fixes Panels API differences)
        Drawing.init();

        turret    = new TurretSubsystemIncremental(hardwareMap);
        vision    = new VisionSubsystem(hardwareMap);
        shooter   = new ShooterSubsystemFF_dualMotor(hardwareMap);
        loader    = new LoaderSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem_Passive_State_new_Incremental(hardwareMap);
        intake    = new IntakeSubsystem(hardwareMap);

        follower.setStartingPose(startPose);
        follower.update();
        buildPaths();

        // Passive spindexer: no hard "homeSlot0AtLoadHere()" assumed.
        // Preload preset so first shoot sequence actually has balls to eject
        spindexer.presetSlots(
                SpindexerSubsystem_Passive_State_new_Incremental.Ball.PURPLE,
                SpindexerSubsystem_Passive_State_new_Incremental.Ball.PURPLE,
                SpindexerSubsystem_Passive_State_new_Incremental.Ball.PURPLE
        );

        telemetry.addLine("BlueAutoDown (converted) ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.addData("PatternTag", vision.getPatternTag());
        telemetry.addData("Aim tx(tag " + TRACK_TAG_ID + ")", "%.2f", vision.getTagTxDegOrNaN(TRACK_TAG_ID));

        telemetry.addData("Spd angle", "%.1f", spindexer.getCurrentAngleDeg());
        telemetry.addData("Spd target", "%.1f", spindexer.getTargetAngleDeg());
        telemetry.addData("Spd full", spindexer.isFull());
        telemetry.addData("Spd anyBall", spindexer.hasAnyBall());
        telemetry.addData("Spd ejecting", spindexer.isEjecting());

        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        turretPhase = TurretPhase.PREAIM_PATTERN;
        turretZeroLatched = false;

        // Intake ON at start (like your current style)
        intake.startIntake();
    }

    @Override
    public void loop() {
        // Keep shooter spinning the whole match (simple + reliable in auto)
        shooter.update(true, false, false, fieldPos);

        follower.update();

        updateAutoPatternTag();

        updateTurretControl();
        turret.update();

        loader.updateLoader();

        if (!shooting) {
            spindexer.update(telemetry, loader, false, autoPatternTag);
        }

        autonomousPathUpdate();

        // update intake slew + stopper logic
        intake.update();

        // Draw robot
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

        PoseStorage.lastPose = follower.getPose();
        PoseStorage.lastTurretAngleDeg = turret.getCurrentAngleDeg();

        telemetry.addData("pathState", pathState);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("AutoPatternTag", autoPatternTag);
        telemetry.addData("TurretPhase", turretPhase);
        telemetry.addData("TurretAngle", "%.1f", turret.getCurrentAngleDeg());
        telemetry.addData("TurretTarget", "%.1f", turret.getTargetAngleDeg());

        telemetry.addData("Shooting", shooting);
        telemetry.addData("yEdgeSent", yEdgeSent);
        telemetry.addData("Spd ejecting", spindexer.isEjecting());
        telemetry.addData("Spd curAng", "%.1f", spindexer.getCurrentAngleDeg());
        telemetry.addData("Spd tgtAng", "%.1f", spindexer.getTargetAngleDeg());

        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.lastPose = follower.getPose();
        PoseStorage.lastTurretAngleDeg = turret.getCurrentAngleDeg();

        if (turret != null) turret.goToAngle(turret.getCurrentAngleDeg());
        if (intake != null) intake.stopIntake();
    }

    // =========================
    // ===== POSE HELPERS =======
    // =========================
    private static double wrapRad(double a) {
        while (a >= Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private boolean atPose(Pose cur, Pose target, double posTolIn, double headTolDeg) {
        double dx = cur.getX() - target.getX();
        double dy = cur.getY() - target.getY();
        double dist = Math.hypot(dx, dy);

        double dhRad = wrapRad(cur.getHeading() - target.getHeading());
        double dhDeg = Math.toDegrees(dhRad);

        return dist <= posTolIn && Math.abs(dhDeg) <= headTolDeg;
    }
}
