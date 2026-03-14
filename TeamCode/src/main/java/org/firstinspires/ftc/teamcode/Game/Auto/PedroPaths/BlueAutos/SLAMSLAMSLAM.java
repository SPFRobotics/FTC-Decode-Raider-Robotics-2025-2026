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

@Autonomous(name = "SLAM SLAM SLAM", group = "Autonomous", preselectTeleOp = "TeleOpMain")
@Configurable
public class SLAMSLAMSLAM extends OpMode {

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
        turret.lockToAngle(297);
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
        public PathChain RunToSlam;
        public PathChain SlamParking;
        public PathChain ShootParking;
        public PathChain RunToSlam2;
        public PathChain SlamParking2;
        public PathChain ShootParking2;
        public PathChain RunToSlam3;
        public PathChain SlamParking3;
        public PathChain ShootParking3;
        public PathChain Leave;

        public Paths(Follower follower) {
            RunToSpikeOne = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(51.581, 25.791),
                                    new Pose(48.116, 35.349)
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
            RunToSlam = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(56.070, 8.651), new Pose(29.162, 9.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            SlamParking = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.070, 8.651),
                                    new Pose(16.328, 9.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ShootParking = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(16.328, 9.000),
                                    new Pose(56.070, 9.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            RunToSlam2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.070, 9.000),
                                    new Pose(29.162, 9.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            SlamParking2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.070, 9.000),
                                    new Pose(16.328, 9.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ShootParking2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(16.328, 9.000),
                                    new Pose(56.070, 9.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            RunToSlam3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.070, 9.000),
                                    new Pose(29.162, 9.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            SlamParking3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.070, 9.000),
                                    new Pose(16.328, 9.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ShootParking3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(16.328, 9.000),
                                    new Pose(56.070, 9.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.070, 9.000),
                                    new Pose(49.192, 26.185)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
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
                    prepareForIntake();
                    follower.followPath(paths.SlamParking, INTAKE_SPEED, true);
                    setPathState(5);
                }
                break;

            // ---- Slam 1 ----
            case 5: // Intake during slam path 1
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
                    setPathState(6);
                }
                break;

            case 6: // Shoot slam 1
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
                    prepareForIntake();
                    follower.followPath(paths.SlamParking2, INTAKE_SPEED, true);
                    setPathState(7);
                }
                break;

            // ---- Slam 2 ----
            case 7: // Intake during slam path 2
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.ShootParking2, true);
                    setPathState(8);
                }
                break;

            case 8: // Shoot slam 2
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
                    prepareForIntake();
                    follower.followPath(paths.SlamParking3, INTAKE_SPEED, true);
                    setPathState(9);
                }
                break;

            // ---- Slam 3 ----
            case 9: // Intake during slam path 3
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.ShootParking3, true);
                    setPathState(10);
                }
                break;

            case 10: // Shoot slam 3
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
                    setPathState(11);
                }
                break;

            case 11: // Leave for points
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
