package org.firstinspires.ftc.teamcode.Game.Auto.OldPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DualColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Assets.PedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;

import java.io.PrintWriter;

@Autonomous(name = "BS 12 Sorted", group = "Autonomous", preselectTeleOp = "TeleOpMain")
@Configurable
public class Short12Sorted extends OpMode {

    private static final double SHOOT_RPM = Outtake.OuttakeConfig.closeRPM;
    private static final double INTAKE_SPEED = .3;

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

    // Preloaded ball colors in slot order (0, 1, 2).
    private static final String PRELOAD_COLORS = "GPP";

    private int shotsFired = 0;
    private int ballsLoaded = 0;
    private int lastKickerCycles = 0;
    private boolean waitingForSpindexAlign = false;
    private boolean shootingPrepared = false;
    private boolean flywheelStarted = false;

    //In order for feature that will unjam to ball to work correctly, it must be run in a loop. This variable is only used to enable and disable the intake and nothing more.
    private boolean intakeEnabled = false;

    ElapsedTime timer = null;

    private PrintWriter pen = null;

    private ElapsedTime override = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.000, 134.442, Math.toRadians(90)));
        paths = new Paths(follower);
        intake = new Intake(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        outtake = new Outtake(hardwareMap, kicker);
        colorSensor = new DualColorFetch(hardwareMap);
        leds = new LedLights(hardwareMap);
        limelight = new Limelight(hardwareMap);
        turret = new Turret(hardwareMap, true,limelight);
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
        turret.setInitialAngle(0);
    }

    @Override
    public void init_loop() {
        int id = limelight.getMotifId();
        if (id != -1) {
            detectedMotifId = id;
            panelsTelemetry.debug("Detected Motif", detectedMotifId);

        }
        else {
            detectedMotifId = 21;
            panelsTelemetry.addLine("No Motif Found");

        }

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        timer = new ElapsedTime();
        pathState = 0;
        shotsFired = 0;
        ballsLoaded = 3;
        lastKickerCycles = 0;

        if (detectedMotifId == -1) {
            detectedMotifId = 21;
        }
        limelight.start();

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
        follower.followPath(paths.shootBallOne, true);
        //UpdateSpindex updateSpindex = new UpdateSpindex(spindex);
        //updateSpindex.start();
    }

    public void stop(){
        PoseStorage.savePose(follower.getPose());
        PoseStorage.blueAlliance = true;
        PoseStorage.redAlliance = false;
        PoseStorage.setTurretStartPos(turret.getCurrentAngularPosition());
        //spindex.exitProgram();
    }

    @Override
    public void loop() {
        ElapsedTime time = new ElapsedTime();
        if (intakeEnabled) {
            intake.intakeOn(true);
        }
        else {
            intake.intakeOff();
        }
        follower.update();
        leds.cycleColors(10);
        turret.lockToAngle(pathState >= 8 ? 345 : 315);
        autonomousPathUpdate();
        updateSpindexPosition();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shots Fired", shotsFired);
        panelsTelemetry.debug("Balls Loaded", ballsLoaded);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("RPM", outtake.getRPM());
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.debug("Loop Time", time.milliseconds());
        panelsTelemetry.debug("Error", spindex.getError());
        panelsTelemetry.debug("Motif ID", detectedMotifId);
        panelsTelemetry.debug("Sort State", spindex.getAutoSortStateName());
        panelsTelemetry.debug("Pattern Pos", spindex.getSortPatternIndex());
        panelsTelemetry.debug("Slot Colors", "" + spindex.getSlotColors()[0] + spindex.getSlotColors()[1] + spindex.getSlotColors()[2]);
        panelsTelemetry.debug("Turret Pos", turret.getCurrentPosition());
        panelsTelemetry.debug("Turret Target", turret.getTargetPosition());

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shots Fired", shotsFired);
        telemetry.addData("Balls Loaded", ballsLoaded);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("RPM", outtake.getRPM());
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.addData("Loop Time", time.milliseconds());
        telemetry.addData("Error", spindex.getError());
        telemetry.addData("Motif ID", detectedMotifId);
        telemetry.addData("Sort State", spindex.getAutoSortStateName());
        telemetry.addData("Pattern Pos", spindex.getSortPatternIndex());
        telemetry.addData("Slot Colors", "" + spindex.getSlotColors()[0] + spindex.getSlotColors()[1] + spindex.getSlotColors()[2]);
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("Turret Target", turret.getTargetPosition());
        telemetry.addData("Turret is Busy:",turret.isTurretAtTarget());
        telemetry.addData("Spindex is busy?:", spindex.isBusy());
        telemetry.addData("RPM", outtake.getRPM());

        //)
        //panelsTelemetry.update(telemetry);

        telemetry.addLine("Timer: " + timer.milliseconds());
        telemetry.update();
    }

    private void updateSpindexPosition() {
        if (spindex.isOuttakeing()) {
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()]);
        } else {
            spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()]);
        }
    }

    private boolean shootBalls() {
        spindex.setMode(true);
        // If waiting for spindex to align after advancing
        if (waitingForSpindexAlign) {
            if (!spindex.isBusy()) {
                // Spindex reached new position, reset timer and resume shooting
                outtake.resetKickerCycle();
                lastKickerCycles = 0;
                waitingForSpindexAlign = false;
                //override.reset();
            }
            return false;
        }

        // Only run kicker cycle when aligned
        if (!spindex.isBusy()) {
            outtake.enableSpindexKickerCycle(true, SHOOT_RPM);
        }

        // When a shot completes, advance to next slot
        int currentCycles = outtake.getKickerCycleCount();
        if (currentCycles > lastKickerCycles) {
            lastKickerCycles = currentCycles;
            shotsFired++;
            spindex.clearBall(spindex.getIndex());

            if (shotsFired >= 3) {
                shotsFired = 0;
                outtake.resetKickerCycle();
                lastKickerCycles = 0;
                waitingForSpindexAlign = false;
                return true;
            }
            // Advance to next slot and wait for alignment
            spindex.addIndex();
            waitingForSpindexAlign = true;
        }

        return false;
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
        System.out.printf("short12Sorted: Prepare for shooting%n");
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

        public PathChain shootBallOne;
        public PathChain RunToRowOne;
        public PathChain intakeRowOne;
        public PathChain shootRowOne;
        public PathChain RuntoRowTwo;
        public PathChain intakeRowTwo;
        public PathChain shootRowTwo;
        public PathChain RuntoRowThree;
        public PathChain intakeRowThree;
        public PathChain shootRowThree;
        public PathChain Leave;

        public Paths(Follower follower) {
            shootBallOne = follower
                    .pathBuilder().setGlobalDeceleration()
                    .addPath(
                            new BezierCurve(
                                    new Pose(33.638, 133.314),
                                    new Pose(33.638, 109.321),
                                    new Pose(48.251, 95.336)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            RunToRowOne = follower
                    .pathBuilder().setGlobalDeceleration()
                    .addPath(
                            new BezierLine(new Pose(48.251, 95.336), new Pose(42.196, 84.299))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intakeRowOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.196, 84.299), new Pose(27.155, 84.299))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootRowOne = follower
                    .pathBuilder().setGlobalDeceleration()
                    .addPath(
                            new BezierLine(new Pose(27.155, 84.299), new Pose(49.856, 95.336))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            RuntoRowTwo = follower
                    .pathBuilder().setGlobalDeceleration()
                    .addPath(
                            new BezierCurve(
                                    new Pose(49.856, 95.336),
                                    new Pose(54.177, 71.465),
                                    new Pose(42.196, 59.786)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intakeRowTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.196, 59.786), new Pose(27.155, 59.786))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootRowTwo = follower
                    .pathBuilder().setGlobalDeceleration()
                    .addPath(
                            new BezierCurve(
                                    new Pose(27.155, 59.786),
                                    new Pose(43.886, 75.934),
                                    new Pose(48.251, 95.336)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            RuntoRowThree = follower
                    .pathBuilder().setGlobalDeceleration()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.251, 95.336),
                                    new Pose(53.277, 54.277),
                                    new Pose(42.196, 35.941)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intakeRowThree = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.196, 35.941), new Pose(27.155, 35.941))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootRowThree = follower
                    .pathBuilder().setGlobalDeceleration()
                    .addPath(
                            new BezierCurve(
                                    new Pose(27.155, 35.941),
                                    new Pose(43.823, 57.631),
                                    new Pose(54.963, 108.203)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(54.963, 108.203),
                                    new Pose(54.963, 115.066),
                                    new Pose(48.251, 115.066)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to first shoot position (flywheel already spinning from start())
                if (!follower.isBusy()) {
                    prepareForShooting();
                    pathState = 1;
                    //override.reset();
                }
                break;

            case 1: // Shoot 3 preloaded balls (sorted by motif)
                spindex.autoSort(outtake, detectedMotifId, turret);
                System.out.printf("short12Sorted: case1 up%n");
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    System.out.printf("short12Sorted: case1- autosort completed%n");
                    follower.followPath(paths.RunToRowOne, true);
                    pathState = 2;
                }
                break;

            case 2: // Run to row 1 intake position
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.intakeRowOne, INTAKE_SPEED, true);
                    pathState = 3;
                }
                break;

            case 3: // Intake row 1 (slow) - pre-spin flywheel during intake
                runIntakeByDistance();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.shootRowOne, true);
                    pathState = 4;
                }
                break;

            case 4: // Move to shoot position + shoot row 1 balls (sorted)
                System.out.printf("short12Sorted: case 4 up%n");
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy()) {
                    spindex.autoSort(outtake, detectedMotifId, turret,"PPG");
                    System.out.printf("short12Sorted: case4 follower not busy%n");
                }
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    shootingPrepared = false;
                    System.out.printf("short12Sorted: case 4 auto sort complete%n");
                    follower.followPath(paths.RuntoRowTwo, true);
                    pathState = 5;
                }
                break;

            case 5: // Run to row 2 intake position
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.intakeRowTwo, INTAKE_SPEED, true);
                    pathState = 6;
                }
                break;

            case 6: // Intake row 2 (slow) - pre-spin flywheel during intake
                runIntakeByDistance();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.shootRowTwo, true);
                    pathState = 7;
                }
                break;

            case 7: // Move to shoot position + shoot row 2 balls (sorted)
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy()) {
                    spindex.autoSort(outtake, detectedMotifId, turret,"PGP");
                }
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    shootingPrepared = false;
                    follower.followPath(paths.RuntoRowThree, true);
                    pathState = 8;
                }
                break;

            case 8: // Run to row 3 intake position
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.intakeRowThree, INTAKE_SPEED, true);
                    pathState = 9;
                }
                break;

            case 9: // Intake row 3 (slow) - pre-spin flywheel during intake
                runIntakeByDistance();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.shootRowThree, true);
                    pathState = 10;
                }
                break;

            case 10: // Move to shoot position + shoot row 3 balls (sorted)
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy()) {
                    spindex.autoSort(outtake, detectedMotifId, turret,"GPP");
                }
                if (spindex.isAutoSortComplete()) {
                    spindex.resetAutoSort();
                    shootingPrepared = false;
                    follower.followPath(paths.Leave, true);
                    pathState = 11;
                }
                break;

            case 11: // Leave for points
                if (!follower.isBusy()) {
                    pathState = 12;
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
}
