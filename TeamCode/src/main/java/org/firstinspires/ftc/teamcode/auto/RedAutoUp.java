package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;

@Autonomous(name = "Example Auto", group = "Examples")
public class RedAutoUp extends OpMode {

    // ============================
    // ===== TURRET TRACKING ======
    // ============================
    public static boolean ENABLE_TURRET_TRACKING = true;

    public static int TRACK_TAG_ID = 24;
    public static double TRACK_KP = 0.02;           // power per deg of tx
    public static double TRACK_MAX_POWER = 0.25;    // clamp
    public static double TRACK_TX_DEADBAND = 0.4;   // deg
    public static double TRACK_SIGN = 1.0;          // set to -1.0 if turret turns the wrong way

    // ============================
    // ===== PEDRO / PANELS =======
    // ============================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private FieldManager panelsField;
    private static final double ROBOT_RADIUS = 9;

    // ============================
    // ===== SUBSYSTEMS ===========
    // ============================
    private TurretSubsystem turret;
    private VisionSubsystem vision;

    // ============================
    // ===== POSES / PATHS ========
    // ============================
    private final Pose startPose   = new Pose(109, 134, Math.toRadians(90));
    private final Pose scorePose   = new Pose(87, 87, Math.toRadians(45));
    private final Pose pickup1Pose = new Pose(19, 84, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(17.7, 59, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(17.7, 35, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    // ============================
    // ===== PATH BUILDING ========
    // ============================
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup1Pose, scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup2Pose, scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(scorePose, pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup3Pose, scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    // ============================
    // ===== STATE MACHINE ========
    // ============================
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
                }
                break;

            default:
                // do nothing
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // ============================
    // ===== TURRET TRACKING ======
    // ============================
    private void updateTurretTracking() {
        if (!ENABLE_TURRET_TRACKING) {
            turret.setManualPower(0.0);
            return;
        }

        double tx = vision.getTagTxDegOrNaN(TRACK_TAG_ID);

        // If we don't see tag 24, stop turret
        if (Double.isNaN(tx) || Math.abs(tx) < TRACK_TX_DEADBAND) {
            turret.setManualPower(0.0);
            return;
        }

        double power = TRACK_SIGN * TRACK_KP * tx;
        power = Range.clip(power, -TRACK_MAX_POWER, TRACK_MAX_POWER);

        turret.setManualPower(power);
    }

    // ============================
    // ===== OPMODE LIFECYCLE =====
    // ============================
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        follower.setStartingPose(startPose);
        follower.update();

        buildPaths();

        telemetry.addLine("ExampleAuto ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // optional: show tags while waiting
        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // keep turret tracking the entire time (optional toggle at top)
        updateTurretTracking();
        turret.update();

        drawRobotOnPanels(follower.getPose());
        autonomousPathUpdate();

        telemetry.addData("pathState", pathState);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("Seen Tags", vision.getSeenTagIdsString());
        telemetry.addData("tx(tag " + TRACK_TAG_ID + ")", "%.2f", vision.getTagTxDegOrNaN(TRACK_TAG_ID));
        telemetry.update();
    }

    @Override
    public void stop() {
        // safety stop
        if (turret != null) turret.setManualPower(0.0);
    }

    // ============================
    // ===== PANELS DRAWING =======
    // ============================
    private void drawRobotOnPanels(Pose pose) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) return;

        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(ROBOT_RADIUS);

        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.line(pose.getX() + v.getXComponent(), pose.getY() + v.getYComponent());

        panelsField.update();
    }
}
