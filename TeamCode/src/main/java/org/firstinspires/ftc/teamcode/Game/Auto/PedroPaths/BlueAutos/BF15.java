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

@Autonomous(name = "Blue Far 15", group = "Autonomous", preselectTeleOp = "TeleOpMain")
@Configurable
public class BF15 extends OpMode {

    private static final double SHOOT_RPM = Outtake.OuttakeConfig.farRPM;
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
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(180)));
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
        turret.setInitialAngle(270);
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
        turret.lockToAngle(291);
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

    public static class Paths {
        public PathChain RunToSpikeOne;
        public PathChain IntakeSpikeOne;
        public PathChain ShootSpikeOne;
        public PathChain RunToHumanSpike;
        public PathChain IntakeHumanSpike;
        public PathChain ContinueHumanSpikeIntake;
        public PathChain ShootHumanSpike;
        public PathChain RunToHailMary;
        public PathChain IntakeHailMary;
        public PathChain ShootHailMary;
        public PathChain RunToHailMary2;
        public PathChain IntakeHailMary2;
        public PathChain ShootHailMary2;
        public PathChain Leave;

        public Paths(Follower follower) {
            RunToSpikeOne = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(51.581, 25.791),
                                    new Pose(41.116, 35.349)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            IntakeSpikeOne = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(41.116, 35.349),
                                    new Pose(24.791, 35.674)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ShootSpikeOne = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(24.791, 35.674),
                                    new Pose(56.070, 8.651)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            RunToHumanSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.070, 8.651),
                                    new Pose(12.163, 33.023)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                    .build();

            IntakeHumanSpike = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(12.163, 33.023),
                                    new Pose(11.829, 11.465)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(225))
                    .build();

            ContinueHumanSpikeIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.829, 11.465),
                                    new Pose(9.717, 7.452)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                    .build();

            ShootHumanSpike = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.717, 7.452),
                                    new Pose(30.372, 13.605),
                                    new Pose(56.535, 8.581)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            RunToHailMary = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.535, 8.581),
                                    new Pose(11.000, 22.279)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            IntakeHailMary = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.000, 22.279),
                                    new Pose(11.000, 33.256)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            ShootHailMary = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.000, 33.256),
                                    new Pose(56.302, 8.930)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            RunToHailMary2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.302, 8.930),
                                    new Pose(10.558, 28.628)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            IntakeHailMary2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.558, 28.628),
                                    new Pose(10.349, 32.907)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ShootHailMary2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.349, 32.907),
                                    new Pose(57.070, 9.209)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(57.070, 9.209),
                                    new Pose(10.465, 39.116)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Shoot 3 preloaded balls at starting position (GPP sorted)
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
                    follower.followPath(paths.IntakeSpikeOne, INTAKE_SPEED, true);
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
                    follower.followPath(paths.RunToHumanSpike, true);
                    setPathState(5);
                }
                break;

            case 5: // Run to human spike, then start first intake segment
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.IntakeHumanSpike, INTAKE_SPEED, true);
                    setPathState(6);
                }
                break;

            case 6: // Intake human spike first segment
                runIntakeByDistance();
                if (!follower.isBusy()) {
                    follower.followPath(paths.ContinueHumanSpikeIntake, INTAKE_SPEED, true);
                    setPathState(7);
                }
                break;

            case 7: // Continue human spike intake, pre-spin flywheel
                runIntakeByDistance();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.ShootHumanSpike, true);
                    setPathState(8);
                }
                break;

            case 8: // Shoot human spike balls sorted (PGP)
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
                    follower.followPath(paths.RunToHailMary, true);
                    setPathState(9);
                }
                break;

            case 9: // Run to hail mary 1, then start intake
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.IntakeHailMary, INTAKE_SPEED, true);
                    setPathState(10);
                }
                break;

            case 10: // Intake hail mary 1 with color sensor, pre-spin flywheel
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.ShootHailMary, true);
                    setPathState(11);
                }
                break;

            case 11: // Shoot hail mary 1 (color sensor detected order)
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
                    follower.followPath(paths.RunToHailMary2, true);
                    setPathState(12);
                }
                break;

            case 12: // Run to hail mary 2, then start intake
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.IntakeHailMary2, INTAKE_SPEED, true);
                    setPathState(13);
                }
                break;

            case 13: // Intake hail mary 2 with color sensor, pre-spin flywheel
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.ShootHailMary2, true);
                    setPathState(14);
                }
                break;

            case 14: // Shoot hail mary 2 (color sensor detected order)
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
                    setPathState(15);
                }
                break;

            case 15: // Leave for points
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;

            case 16: // Done
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
