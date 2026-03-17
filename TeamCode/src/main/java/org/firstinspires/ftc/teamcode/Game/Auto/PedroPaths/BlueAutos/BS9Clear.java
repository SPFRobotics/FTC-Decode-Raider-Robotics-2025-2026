package org.firstinspires.ftc.teamcode.Game.Auto.PedroPaths.BlueAutos;

import static org.firstinspires.ftc.teamcode.Subsystems.PoseStorage.IntakeSpeed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.DualColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Assets.PedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;

@Autonomous(name = "Blue Short 9", group = "BlueAutos", preselectTeleOp = "Tele-Op Blue")
@Configurable
public class BS9Clear extends OpMode {

    private static final double SHOOT_RPM = Outtake.OuttakeConfig.closeRPM;
    private static final double INTAKE_SPEED = IntakeSpeed;

    private TelemetryManager panelsTelemetry;

    public Follower follower;
    private Paths paths;
    private int pathState;

    private Spindex spindex;
    private Outtake outtake;
    private Intake intake;
    private KickerSpindex kicker;
    private DualColorFetch colorSensor;
    private LedLights leds = null;
    private Limelight limelight;
    private Turret turret;
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
        follower.setStartingPose(new Pose(33.400, 133.900, Math.toRadians(180)));
        paths = new Paths(follower);
        pathTimer = new ElapsedTime();
        intake = new Intake(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        outtake = new Outtake(hardwareMap, kicker);
        colorSensor = new DualColorFetch(hardwareMap);
        leds = new LedLights(hardwareMap);
        limelight = new Limelight(hardwareMap);
        turret = new Turret(hardwareMap, true, limelight);
        spindex = new Spindex(hardwareMap);

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
        turret.setInitialAngle(180);
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
        if (outtake.isFarLocation()) {
            outtake.switchLocation();
        }
        outtake.setRPM(SHOOT_RPM);

        for (int i = 0; i < 3; i++) {
            spindex.setIndex(i);
            spindex.setSlotColor(PRELOAD_COLORS.charAt(i));
        }
        spindex.setIndex(0);
        spindex.setMode(true);
        follower.followPath(paths.RunToShootPreload, true);
    }

    @Override
    public void stop() {
        PoseStorage.savePose(follower.getPose());
        PoseStorage.blueAlliance = true;
        PoseStorage.redAlliance = false;
        PoseStorage.setTurretStartPos(turret.getCurrentAngularPosition());
    }

    @Override
    public void loop() {
        if (intakeEnabled) {
            intake.intakeOn(true);
        } else {
            intake.intakeOff();
        }
        follower.update();
        leds.cycleColors(10);
        turret.lockToAngle(Turret.TurretConfig.turretShortLockLine);
        autonomousPathUpdate();
        updateSpindexPosition();
    }

    private void updateSpindexPosition() {
        if (spindex.isOuttakeing()) {
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()]);
        } else {
            spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()]);
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

        public PathChain RunToShootPreload;
        public PathChain RunToSpikeTwo;
        public PathChain IntakeSpikeTwo;
        public PathChain ClearRamp;
        public PathChain RunToShootSpikeTwo;
        public PathChain RunToSpikeOne;
        public PathChain IntakeSpikeOne;
        public PathChain RunToShootSpikeOne;
        public PathChain Leave;

        public Paths(Follower follower) {
            RunToShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(33.400, 133.900),
                                    new Pose(36.749, 105.565),
                                    new Pose(47.776, 95.823)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            RunToSpikeTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.776, 95.823),
                                    new Pose(52.982, 72.476),
                                    new Pose(44.234, 60.164)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            IntakeSpikeTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.234, 60.164), new Pose(15.000, 57.950))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ClearRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.000, 57.950),
                                    new Pose(54.465, 63.292),
                                    new Pose(17.435, 70.258)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            RunToShootSpikeTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(17.435, 70.258),
                                    new Pose(79.396, 70.258),
                                    new Pose(49.064, 95.823)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            RunToSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(49.064, 95.823), new Pose(45.700, 84.108))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            IntakeSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.700, 84.108), new Pose(24.200, 84.108))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            RunToShootSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.200, 84.108),
                                    new Pose(39.322, 80.026),
                                    new Pose(47.776, 95.823)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.776, 95.823), new Pose(23.000, 74.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to shoot position (flywheel already spinning from start())
                if (!follower.isBusy()) {
                    prepareForShooting();
                    setPathState(1);
                }
                break;

            case 1: // Shoot 3 preloaded balls (sorted by motif)
                spindex.autoSort(outtake, detectedMotifId, turret, "GPP");
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    follower.followPath(paths.RunToSpikeTwo, true);
                    setPathState(2);
                }
                break;

            case 2: // Run to spike two intake position
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.IntakeSpikeTwo, INTAKE_SPEED, true);
                    setPathState(3);
                }
                break;

            case 3: // Intake spike two (slow), pre-spin flywheel
                runIntakeByDistance();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    intakeEnabled = false;
                    follower.followPath(paths.ClearRamp,.7, true);
                    setPathState(4);
                }
                break;

            case 4: // Clear ramp (intake off), wait for completion
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5: // Settle after ramp clear, then drive to shoot
                if (pathTimer.milliseconds() > 500) {
                    intakeEnabled = true;
                    follower.followPath(paths.RunToShootSpikeTwo, true);
                    setPathState(6);
                }
                break;

            case 6: // Shoot spike two balls (sorted)
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy()) {
                    spindex.autoSort(outtake, detectedMotifId, turret, "PGP");
                }
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    shootingPrepared = false;
                    follower.followPath(paths.RunToSpikeOne, true);
                    setPathState(7);
                }
                break;

            case 7: // Run to spike one intake position
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.IntakeSpikeOne, INTAKE_SPEED, true);
                    setPathState(8);
                }
                break;

            case 8: // Intake spike one (slow), pre-spin flywheel
                runIntakeByDistance();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.RunToShootSpikeOne, true);
                    setPathState(9);
                }
                break;

            case 9: // Shoot spike one balls (sorted)
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy()) {
                    spindex.autoSort(outtake, detectedMotifId, turret, "PPG");
                }
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    shootingPrepared = false;
                    follower.followPath(paths.Leave, true);
                    setPathState(10);
                }
                break;

            case 10: // Leave for points
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11: // Done
                outtake.setRPM(0);
                turret.setPower(0);
                intakeEnabled = false;
                kicker.down();
                requestOpModeStop();
                break;

            default:
                outtake.setRPM(0);
                turret.setPower(0);
                intakeEnabled = false;
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
