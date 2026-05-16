package org.firstinspires.ftc.teamcode.Game.Auto.PedroPaths.RedAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Assets.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.DualColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextIntake;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextOuttake;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextTurret;

@Autonomous(name = "Red Far 12", group = "RedAutos", preselectTeleOp = "Tele-Op Red")
@Configurable
public class SLAMSLAMSLAM extends OpMode {

    private static final double SHOOT_RPM = NextOuttake.farRPM;
    private static final double INTAKE_SPEED = .7;

    private TelemetryManager panelsTelemetry;

    public Follower follower;
    private Paths paths;
    private int pathState;

    private NextSpindex spindex = NextSpindex.INSTANCE;
    private NextOuttake outtake = NextOuttake.INSTANCE;
    private NextIntake intake = NextIntake.INSTANCE;
    private KickerSpindex kicker;
    private DualColorFetch colorSensor;
    private LedLights leds = null;
    private Limelight limelight;
    private NextTurret turret = NextTurret.INSTANCE;
    private int detectedMotifId = -1;

    private static final String PRELOAD_COLORS = "GPP";

    private int ballsLoaded = 0;
    private boolean shootingPrepared = false;
    private boolean flywheelStarted = false;
    private boolean intakeEnabled = false;

    ElapsedTime timer = null;
    private ElapsedTime pathTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(0)));
        paths = new Paths(follower);
        pathTimer = new ElapsedTime();
        intake.initialize(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        outtake.setKicker(kicker);
        outtake.initialize(hardwareMap);
        colorSensor = new DualColorFetch(hardwareMap);
        leds = new LedLights(hardwareMap);
        limelight = new Limelight(hardwareMap);
        turret.setGoalCoords(false);
        turret.setLimelight(limelight);
        turret.initialize(hardwareMap);
        spindex.initialize(hardwareMap);

        spindex.setAutoSortActive(true);
        turret.setAlignmentEnabled(true);
        spindex.setAutoLoadMode(true);
        outtake.resetKickerCycle();
        kicker.down();

        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();
        telemetry.setMsTransmissionInterval(16);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        turret.setInitialAngle(90);
    }

    @Override
    public void init_loop() {
        int id = limelight.getMotifId();
        if (id != -1) {
            detectedMotifId = id;
        }

        panelsTelemetry.debug("Raw Limelight ID", id);
        panelsTelemetry.debug("Stored Motif", detectedMotifId);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        timer = new ElapsedTime();
        pathTimer = new ElapsedTime();
        pathState = 0;
        ballsLoaded = 3;

        if (detectedMotifId == -1) {
            detectedMotifId = 21;
        }
        limelight.stop();

        intakeEnabled = true;
        if (!outtake.isFarLocation()) {
            outtake.switchLocation();
        }
        outtake.setRPM(SHOOT_RPM);

        for (int i = 0; i < 3; i++) {
            spindex.setIndex(i);
            spindex.setSlotColor(PRELOAD_COLORS.charAt(i));
        }
        spindex.setIndex(0);
        spindex.setMode(true);
    }

    @Override
    public void stop() {
        PoseStorage.savePose(follower.getPose());
        PoseStorage.blueAlliance = false;
        PoseStorage.redAlliance = true;
        PoseStorage.setTurretStartPos(turret.getCurrentAngularPosition());
    }

    @Override
    public void loop() {
        if (intakeEnabled) {
            intake.turnOn();
        } else {
            intake.turnOff();
        }
        intake.periodic();
        outtake.periodic();
        spindex.periodic();
        follower.update();
        leds.cycleColors(10);
        turret.lockToAngle(69);
        turret.periodic();
        autonomousPathUpdate();
        updateSpindexPosition();
    }

    private void updateSpindexPosition() {
        if (spindex.isOuttakeing()) {
            spindex.moveToPos(NextSpindex.outtakePos[spindex.getIndex()]);
        } else {
            spindex.moveToPos(NextSpindex.intakePos[spindex.getIndex()]);
        }
    }

    private void runIntake() {
        spindex.setMode(false);
        spindex.autoLoad(colorSensor);

        int loadedCount = 0;
        for (char slot : spindex.getSlotColors()) {
            if (slot != 'E') loadedCount++;
        }
        ballsLoaded = loadedCount;
    }

    private void runIntakeByDistance() {
        spindex.setMode(false);
        spindex.autoLoadByDistance(colorSensor);

        int loadedCount = 0;
        for (char slot : spindex.getSlotColors()) {
            if (slot != 'E') loadedCount++;
        }
        ballsLoaded = loadedCount;
    }

    private void prepareForIntake() {
        spindex.setMode(false);
        spindex.setIndex(0);
        spindex.resetAutoSort();
        ballsLoaded = 0;
        for (int i = 0; i < 3; i++) {
            spindex.clearBall(i);
        }
    }

    private void prepareForShooting() {
        outtake.setRPM(SHOOT_RPM);
        outtake.resetKickerCycle();
        spindex.setAutoSortActive(true);
    }

    /* ============================================================= *
     *        Pedro Pathing Plus Visualizer — Auto-Generated         *
     *                                                               *
     *  Version: 1.7.5.                                              *
     *  Copyright (c) 2026 Matthew Allen                             *
     *                                                               *
     *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
     *  Changes will be overwritten when regenerated.                *
     * ============================================================= */

    public static class Paths {

        public PathChain RunToSpikeOne;
        public PathChain IntakeSpikeOne;
        public PathChain ShootSpikeOne;
        public PathChain RunToSlam;
        public PathChain SlamParking;
        public PathChain Reverse;
        public PathChain ReSlam;
        public PathChain ShootParking;
        public PathChain Leave;

        public Paths(Follower follower) {
            RunToSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.000, 8.000),
                                    new Pose(92.419, 25.791),
                                    new Pose(95.884, 35.349)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            IntakeSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.884, 35.349), new Pose(119.209, 35.674))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            ShootSpikeOne = follower
                    .pathBuilder()
                    .setGlobalDeceleration()
                    .addPath(
                            new BezierLine(new Pose(119.209, 35.674), new Pose(87.930, 8.651))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            RunToSlam = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.930, 8.651), new Pose(117.240, 11.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            SlamParking = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(117.240, 11.000), new Pose(127.672, 11.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Reverse = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(127.672, 11.000), new Pose(107.601, 11.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            ReSlam = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(107.601, 11.000), new Pose(144-9.872784150156415, 11.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(330))
                    .build();

            ShootParking = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-9.872784150156415, 11.000), new Pose(87.930, 11.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(330),Math.toRadians(0))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.930, 11.000), new Pose(94.808, 26.185))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Shoot 3 preloaded balls (GPP sorted)
                prepareForShooting();
                setPathState(1);
                break;

            case 1: // autoSort preloaded GPP
                spindex.autoSort(outtake, detectedMotifId, turret);
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    follower.followPath(paths.RunToSpikeOne, true);
                    setPathState(2);
                }
                break;

            case 2: // Run to spike one intake position
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.IntakeSpikeOne, .3, true);
                    setPathState(3);
                }
                break;

            case 3: // Intake spike one (slow), pre-spin flywheel
                runIntakeByDistance();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.ShootSpikeOne, true);
                    setPathState(4);
                }
                break;

            case 4: // Shoot spike one balls sorted (GPP)
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy()) {
                    spindex.autoSort(outtake, detectedMotifId, turret, "GPP");
                }
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    shootingPrepared = false;
                    follower.followPath(paths.RunToSlam, true);
                    setPathState(5);
                }
                break;

            // ---- Slam Cycle 1 ----
            case 5: // RunToSlam done, start SlamParking intake
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.SlamParking, INTAKE_SPEED, true);
                    setPathState(6);
                }
                break;

            case 6: // Intake during SlamParking
                runIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Reverse, true);
                    setPathState(7);
                }
                break;

            case 7: // Reverse done, start ReSlam intake
                if (!follower.isBusy()) {
                    follower.followPath(paths.ReSlam, INTAKE_SPEED, true);
                    setPathState(8);
                }
                break;

            case 8: // Intake during ReSlam, pre-spin flywheel
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.ShootParking, true);
                    setPathState(9);
                }
                break;

            case 9: // Shoot slam cycle 1
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy()) {
                    spindex.autoSort(outtake, detectedMotifId, turret);
                }
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    shootingPrepared = false;
                    follower.followPath(paths.RunToSlam, true);
                    setPathState(10);
                }
                break;

            // ---- Slam Cycle 2 ----
            case 10: // RunToSlam done, start SlamParking intake
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.SlamParking, INTAKE_SPEED, true);
                    setPathState(11);
                }
                break;

            case 11: // Intake during SlamParking
                runIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Reverse, true);
                    setPathState(12);
                }
                break;

            case 12: // Reverse done, start ReSlam intake
                if (!follower.isBusy()) {
                    follower.followPath(paths.ReSlam, INTAKE_SPEED, true);
                    setPathState(13);
                }
                break;

            case 13: // Intake during ReSlam, pre-spin flywheel
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.ShootParking, true);
                    setPathState(14);
                }
                break;

            case 14: // Shoot slam cycle 2
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy()) {
                    spindex.autoSort(outtake, detectedMotifId, turret);
                }
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    shootingPrepared = false;
                    follower.followPath(paths.RunToSlam, true);
                    setPathState(15);
                }
                break;
/*
            // ---- Slam Cycle 3 ----
            case 15: // RunToSlam done, start SlamParking intake
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.SlamParking, INTAKE_SPEED, true);
                    setPathState(16);
                }
                break;

            case 16: // Intake during SlamParking
                runIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Reverse, true);
                    setPathState(17);
                }
                break;

            case 17: // Reverse done, start ReSlam intake
                if (!follower.isBusy()) {
                    follower.followPath(paths.ReSlam, INTAKE_SPEED, true);
                    setPathState(18);
                }
                break;

            case 18: // Intake during ReSlam, pre-spin flywheel
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.ShootParking, true);
                    setPathState(19);
                }
                break;

            case 19: // Shoot slam cycle 3
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy()) {
                    spindex.autoSort(outtake, detectedMotifId, turret);
                }
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    shootingPrepared = false;
                    follower.followPath(paths.Leave, true);
                    setPathState(20);
                }
                break;

 */

            case 20: // Leave for points
                if (!follower.isBusy()) {
                    setPathState(21);
                }
                break;

            case 21: // Done
                outtake.setRPM(0);
                intakeEnabled = false;
                kicker.down();
                requestOpModeStop();
                break;

            default:
                outtake.setRPM(0);
                intakeEnabled = false;
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
