package org.firstinspires.ftc.teamcode.Game.Auto.PedroPaths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.UpdateSpindex;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;

@Autonomous(name = "Red Far", group = "Autonomous")
@Configurable
public class FarRedPath extends OpMode {

    private static final double SHOOT_RPM = Outtake.OuttakeConfig.farRPM;
    private static final double INTAKE_SPEED = 0.25;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private int pathState;

    private Spindex spindex;
    private Outtake outtake;
    private Intake intake;
    private KickerSpindex kicker;
    private ColorFetch colorSensor;
    private LedLights leds = null;

    private int shotsFired = 0;
    private int ballsLoaded = 0;
    private int lastKickerCycles = 0;
    private boolean waitingForSpindexAlign = false;
    private boolean shootingPrepared = false;
    private boolean flywheelStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(90)));
        paths = new Paths(follower);

        spindex = new Spindex(hardwareMap);
        outtake = new Outtake(hardwareMap, true);
        intake = new Intake(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        colorSensor = new ColorFetch(hardwareMap);
        leds = new LedLights(hardwareMap);

        spindex.setAutoLoadMode(true);
        outtake.resetKickerCycle();
        kicker.down();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathState = 0;
        shotsFired = 0;
        ballsLoaded = 0;
        lastKickerCycles = 0;

        intake.setPower(1);
        outtake.setRPM(SHOOT_RPM);
        spindex.setMode(true);  // Pre-position spindex for shooting during travel
        follower.followPath(paths.shootBallOne, true);
        UpdateSpindex updateSpindex = new UpdateSpindex(spindex);
        updateSpindex.start();
    }

    public void stop(){
        spindex.exitProgram();
    }

    @Override
    public void loop() {
        ElapsedTime time = new ElapsedTime();
        follower.update();
        leds.cycleColors(10);
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shots Fired", shotsFired);
        panelsTelemetry.debug("Balls Loaded", ballsLoaded);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("RPM", outtake.getRPM());
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.debug("Loop Time", time.milliseconds());
        panelsTelemetry.update(telemetry);
    }

    private void updateSpindexPosition() {
        if (spindex.isOuttakeing()) {
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], true);
        } else {
            spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], true);
        }
    }

    private boolean shootBalls() {
        spindex.setMode(true);
        // If waiting for spindex to align after advancing
        if (waitingForSpindexAlign) {
            if (spindex.atTarget()) {
                // Spindex reached new position, reset timer and resume shooting
                outtake.resetKickerCycle();
                lastKickerCycles = 0;
                waitingForSpindexAlign = false;
            }
            return false;
        }

        // Only run kicker cycle when aligned
        if (spindex.atTarget()) {
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
        for (boolean slot : spindex.getSlotStatus()) {
            if (slot) loadedCount++;
        }
        ballsLoaded = loadedCount;
    }

    private void prepareForIntake() {
        spindex.setMode(false);
        spindex.setIndex(0);
        ballsLoaded = 0;
        for (int i = 0; i < 3; i++) {
            spindex.clearBall(i);
        }
    }

    private void prepareForShooting() {
        spindex.setMode(true);
        spindex.setIndex(0);
        shotsFired = 0;
        outtake.resetKickerCycle();
        lastKickerCycles = 0;
        waitingForSpindexAlign = false;
        outtake.setRPM(SHOOT_RPM);
    }


    public static class Paths {
        public PathChain shootBallOne;
        public PathChain RunToRowOne;
        public PathChain intakeRowOne;
        public PathChain shootRowOne;
        public PathChain RuntoRowTwo;
        public PathChain intakeRowTwo;
        public PathChain shootRowTwo;
        public PathChain RuntwoRowThree;
        public PathChain IntakeRowThree;
        public PathChain backToShooting;
        public PathChain LeavePoints;

        public Paths(Follower follower) {
            shootBallOne = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),

                                    new Pose(88.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(69))

                    .build();

            RunToRowOne = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 12.000),
                                    new Pose(88.000, 36.000),
                                    new Pose(88.000, 36.000),
                                    new Pose(100.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))

                    .build();

            intakeRowOne = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 36.000),

                                    new Pose(135.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootRowOne = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.000, 36.000),
                                    new Pose(111.000, 36.000),
                                    new Pose(112.000, 12.000),
                                    new Pose(88.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))

                    .build();

            RuntoRowTwo = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 12.000),
                                    new Pose(88.000, 62.000),
                                    new Pose(88.000, 62.000),
                                    new Pose(100.000, 62.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))

                    .build();

            intakeRowTwo = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(100.000, 62.000),
                                    new Pose(112.000, 62.000),
                                    new Pose(111.000, 58.000),
                                    new Pose(135.000, 58.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shootRowTwo = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.000, 58.000),
                                    new Pose(96.000, 58.000),
                                    new Pose(88.201, 47.397),
                                    new Pose(88.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))

                    .build();

            RuntwoRowThree = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 12.000),
                                    new Pose(68.115, 44.023),
                                    new Pose(104.900, 83.653)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))

                    .build();

            IntakeRowThree = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(104.900, 83.653),

                                    new Pose(127.862, 83.598)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            backToShooting = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(127.862, 83.598),
                                    new Pose(68.232, 55.280),
                                    new Pose(87.766, 8.904)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))

                    .build();

            LeavePoints = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.766, 8.904),
                                    new Pose(63.469, 65.805),
                                    new Pose(113.941, 69.259)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to first shoot position (flywheel already spinning from start())
                if (!follower.isBusy()) {
                    prepareForShooting();
                    pathState = 1;
                }
                break;

            case 1: // Shoot 3 preloaded balls
                if (shootBalls()) {
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
                runIntake();
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

            case 4: // Move to shoot position + shoot row 1 balls
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy() && shootBalls()) {
                    shootingPrepared = false;
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
                runIntake();
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

            case 7: // Move to shoot position + shoot row 2 balls
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy() && shootBalls()) {
                    shootingPrepared = false;
                    follower.followPath(paths.RuntwoRowThree, true);
                    pathState = 8;
                }
                break;

            case 8: // Run to row 3 intake position
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.IntakeRowThree, INTAKE_SPEED, true);
                    pathState = 9;
                }
                break;

            case 9: // Intake row 3 (slow) - pre-spin flywheel during intake
                runIntake();
                if (!flywheelStarted) {
                    outtake.setRPM(SHOOT_RPM);
                    flywheelStarted = true;
                }
                if (!follower.isBusy()) {
                    spindex.setMode(true);
                    spindex.setIndex(0);
                    flywheelStarted = false;
                    follower.followPath(paths.backToShooting, true);
                    pathState = 10;
                }
                break;

            case 10: // Move to shoot position + shoot row 3 balls
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy() && shootBalls()) {
                    shootingPrepared = false;
                    follower.followPath(paths.LeavePoints, true);
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
                intake.setPower(0);
                kicker.down();
                requestOpModeStop();
                break;

            default:
                outtake.setRPM(0);
                intake.setPower(0);
                break;
        }
    }
}
