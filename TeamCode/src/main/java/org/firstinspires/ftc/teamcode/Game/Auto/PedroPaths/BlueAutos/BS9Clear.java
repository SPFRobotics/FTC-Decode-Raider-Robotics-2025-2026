package org.firstinspires.ftc.teamcode.Game.Auto.PedroPaths.BlueAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Assets.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Autonomous(name = "BlueShortClear", group = "Autonomous")
@Configurable
public class BS9Clear extends OpMode {

    private static final double SHOOT_RPM = Outtake.OuttakeConfig.closeRPM;
    private static final double INTAKE_SPEED = 0.25;
    private static final String PRELOAD_COLORS = "GPP";

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private int pathState;
    private ElapsedTime pathTimer;

    private Turret turret;
    private Spindex spindex;
    private Outtake outtake;
    private Intake intake;
    private KickerSpindex kicker;
    private ColorFetch colorSensor;
    private LedLights leds;
    private Limelight limelight;

    private int detectedMotifId = -1;
    private int ballsLoaded = 0;
    private boolean shootingPrepared = false;
    private boolean flywheelStarted = false;
    private boolean intakeEnabled = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.400, 133.900, Math.toRadians(180.000)));
        paths = new Paths(follower);
        pathTimer = new ElapsedTime();

        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        outtake = new Outtake(hardwareMap, kicker);
        colorSensor = new ColorFetch(hardwareMap);
        leds = new LedLights(hardwareMap);
        limelight = new Limelight(hardwareMap);
        turret = new Turret(hardwareMap, true);

        spindex.setAutoLoadMode(true);
        outtake.resetKickerCycle();
        kicker.down();

        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();
        telemetry.setMsTransmissionInterval(1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        int id = limelight.getMotifId();
        if (id != -1) {
            detectedMotifId = id;
            panelsTelemetry.debug("Detected Motif", detectedMotifId);
        } else {
            detectedMotifId = 21;
            panelsTelemetry.addLine("No Motif Found");
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathTimer.reset();
        pathState = 0;
        ballsLoaded = 0;

        if (detectedMotifId == -1) {
            detectedMotifId = 21;
        }
        limelight.stop();

        intakeEnabled = true;
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
        ElapsedTime loopTimer = new ElapsedTime();

        if (intakeEnabled) {
            intake.intakeOn(true);
        } else {
            intake.intakeOff();
        }

        follower.update();
        leds.cycleColors(10);
        turret.lockToAngle(45);
        autonomousPathUpdate();
        updateSpindexPosition();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Balls Loaded", ballsLoaded);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("RPM", outtake.getRPM());
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.debug("Turret At Target", turret.isTurretAtTarget());
        panelsTelemetry.debug("Turret Pos", turret.getCurrentPosition());
        panelsTelemetry.debug("Turret Target", turret.getTargetPosition());
        panelsTelemetry.debug("Sort State", spindex.getAutoSortStateName());
        panelsTelemetry.debug("Slot Colors", "" + spindex.getSlotColors()[0] + spindex.getSlotColors()[1] + spindex.getSlotColors()[2]);
        panelsTelemetry.debug("Motif ID", detectedMotifId);
        panelsTelemetry.debug("Loop Time", loopTimer.milliseconds());
        panelsTelemetry.update(telemetry);
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

    public static class Paths {

        public PathChain RunToShootPreload;
        public PathChain RunToSpikeTwo;
        public PathChain IntakeSpikeTwo;
        public PathChain RunToShootSpikeTwo;
        public PathChain RunToSpikeOne;
        public PathChain IntakeSpikeOne;
        public PathChain ClearRamp;
        public PathChain RunToShootSpikeOne;
        public PathChain line9;

        public Paths(Follower follower) {
            RunToShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.400, 133.900), new Pose(47.776, 95.823))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            RunToSpikeTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.776, 95.823), new Pose(47.776, 60.164))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            IntakeSpikeTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.776, 60.164), new Pose(26.262, 60.164))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            RunToShootSpikeTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(26.262, 60.164), new Pose(49.064, 95.823))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            RunToSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(49.064, 95.823), new Pose(33.400, 84.108))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            IntakeSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.400, 84.108), new Pose(25.086, 84.108))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ClearRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(25.086, 84.108), new Pose(16.989, 77.154))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            RunToShootSpikeOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.989, 77.154), new Pose(47.776, 95.823))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            line9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.776, 95.823), new Pose(23.000, 74.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Arrive at shoot preload position
                if (!follower.isBusy()) {
                    prepareForShooting();
                    setPathState(1);
                }
                break;

            case 1: // Shoot 3 preloaded balls sorted — only fire when turret is aimed
                spindex.autoSort(outtake, detectedMotifId, turret);
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    follower.followPath(paths.RunToSpikeTwo, true);
                    setPathState(2);
                }
                break;

            case 2: // Run to spike two
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.IntakeSpikeTwo, INTAKE_SPEED, true);
                    setPathState(3);
                }
                break;

            case 3: // Intake spike two (slow), pre-spin flywheel
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.RunToShootSpikeTwo, true);
                    setPathState(4);
                }
                break;

            case 4: // Move to shoot position + shoot spike two balls (turret-gated)
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
                    follower.followPath(paths.RunToSpikeOne, true);
                    setPathState(5);
                }
                break;

            case 5: // Run to spike one
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.IntakeSpikeOne, INTAKE_SPEED, true);
                    setPathState(6);
                }
                break;

            case 6: // Intake spike one (slow), pre-spin flywheel
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.turnTo(1.571);
                    setPathState(7);
                }
                break;

            case 7: // Wait for turn, then clear ramp
                if (!follower.isBusy()) {
                    follower.followPath(paths.ClearRamp, true);
                    setPathState(8);
                }
                break;

            case 8: // Wait for ramp clear to finish
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;

            case 9: // Brief settle before heading to shoot
                if (pathTimer.milliseconds() > 500) {
                    follower.followPath(paths.RunToShootSpikeOne, true);
                    setPathState(10);
                }
                break;

            case 10: // Move to shoot position + shoot spike one balls (turret-gated)
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
                    follower.followPath(paths.line9, true);
                    setPathState(11);
                }
                break;

            case 11: // Park
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;

            case 12: // Done
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
