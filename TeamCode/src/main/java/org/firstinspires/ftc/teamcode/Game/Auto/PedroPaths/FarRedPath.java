package org.firstinspires.ftc.teamcode.Game.Auto.PedroPaths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.UpdateSpindex;
import org.firstinspires.ftc.teamcode.Resources.PedroPathing.Constants;
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

    //In order for feature that will unjam to ball to work correctly, it must be run in a loop. This variable is only used to enable and disable the intake and nothing more.
    private boolean intakeEnabled = false;

    private ElapsedTime override = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.000, 134.442, Math.toRadians(90)));
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

        intakeEnabled = true;
        outtake.setRPM(SHOOT_RPM);
        spindex.setMode(true);  // Pre-position spindex for shooting during travel
        follower.followPath(paths.shootBallOne, true);
        //UpdateSpindex updateSpindex = new UpdateSpindex(spindex);
        //updateSpindex.start();
    }

    public void stop(){
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
        panelsTelemetry.update(telemetry);
    }

    private void updateSpindexPosition() {
        if (spindex.isOuttakeing()) {
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], 3);
        } else {
            spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], 3);
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
                //override.reset();
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
                                    new Pose(91.738, 29.902),
                                    new Pose(100.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))

                    .build();

            intakeRowOne = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 36.000),

                                    new Pose(125.159, 35.598)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootRowOne = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.159, 35.598),
                                    new Pose(101.311, 30.885),
                                    new Pose(88.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))

                    .build();

            RuntoRowTwo = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 12.000),
                                    new Pose(88.787, 44.295),
                                    new Pose(100.393, 58.852)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))

                    .build();

            intakeRowTwo = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.393, 58.852),

                                    new Pose(125.159, 59.808)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootRowTwo = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.159, 59.808),
                                    new Pose(98.430, 45.234),
                                    new Pose(88.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))

                    .build();

            LeavePoints = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 12.000),
                                    new Pose(95.928, 58.133),
                                    new Pose(115.646, 69.259)
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
                    //override.reset();
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
                    //override.reset();
                }
                // Only shoot once path completes and robot is in position
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
                    //override.reset();
                }
                if (!follower.isBusy() && shootBalls()) {
                    shootingPrepared = false;
                    follower.followPath(paths.LeavePoints, true);
                    pathState = 8;
                }
                break;

            case 8: // Leave for points
                if (!follower.isBusy()) {
                    pathState = 9;
                }
                break;

            case 9: // Done
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
}
