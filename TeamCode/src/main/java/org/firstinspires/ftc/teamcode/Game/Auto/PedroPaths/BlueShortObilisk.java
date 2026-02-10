package org.firstinspires.ftc.teamcode.Game.Auto.PedroPaths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
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

@Autonomous(name = "Blue Short OBL", group = "Autonomous")
@Configurable
public class BlueShortObilisk extends OpMode {

    private static final double SHOOT_RPM = Outtake.OuttakeConfig.closeRPM;
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
    private Limelight limelight;

    private int shotsFired = 0;
    private int ballsLoaded = 0;
    private int lastKickerCycles = 0;
    private boolean waitingForSpindexAlign = false;
    private boolean shootingPrepared = false;
    private boolean flywheelStarted = false;

    private int detectedMotifId = -1;
    private static final double OBELISK_READ_TIMEOUT_MS = 2000;
    private ElapsedTime obeliskReadTimer = new ElapsedTime();

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
        limelight = new Limelight(hardwareMap);

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
        detectedMotifId = -1;

        intake.setPower(1);
        outtake.setRPM(SHOOT_RPM);
        spindex.setMode(true);  // Pre-position spindex for shooting during travel
        follower.followPath(paths.ReadObilisk, true);
    }

    public void stop(){
        //spindex.exitProgram();
    }

    @Override
    public void loop() {
        ElapsedTime time = new ElapsedTime();
        follower.update();
        leds.cycleColors(10);
        autonomousPathUpdate();
        updateSpindexPosition();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Detected Motif", detectedMotifId);
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
        public PathChain ReadObilisk;
        public PathChain shootBallOne;
        public PathChain RunToRowOne;
        public PathChain intakeRowOne;
        public PathChain shootRowOne;
        public PathChain RuntoRowTwo;
        public PathChain intakeRowTwo;
        public PathChain shootRowTwo;
        public PathChain LeavePoints;

        public Paths(Follower follower) {
            ReadObilisk = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(33.176, 134.442),
                                    new Pose(33.431, 115.932),
                                    new Pose(58.644, 113.473)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(78))

                    .build();

            shootBallOne = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.644, 113.473),
                                    new Pose(57.219, 102.810),
                                    new Pose(57.674, 86.442)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(78), Math.toRadians(135))

                    .build();

            RunToRowOne = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.674, 86.442),
                                    new Pose(51.231, 89.286),
                                    new Pose(43.525, 84.432)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            intakeRowOne = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.525, 84.432),

                                    new Pose(16.594, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            shootRowOne = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.594, 84.000),
                                    new Pose(46.804, 84.684),
                                    new Pose(53.281, 81.949),
                                    new Pose(57.488, 86.140)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            RuntoRowTwo = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.488, 86.140),

                                    new Pose(43.154, 59.681)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            intakeRowTwo = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.154, 59.681),

                                    new Pose(17.607, 59.406)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            shootRowTwo = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17.607, 59.406),
                                    new Pose(23.994, 58.006),
                                    new Pose(48.078, 64.834),
                                    new Pose(57.488, 85.953)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            LeavePoints = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.488, 85.953),
                                    new Pose(49.743, 76.020),
                                    new Pose(39.706, 71.911)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Travel to obelisk reading position
                if (!follower.isBusy()) {
                    obeliskReadTimer.reset();
                    pathState = 1;
                }
                break;

            case 1: // Read obelisk using Limelight
                int motifId = limelight.getMotifId();
                if (motifId != -1) {
                    detectedMotifId = motifId;
                    panelsTelemetry.debug( "Motif ID: " + detectedMotifId);
                    follower.followPath(paths.shootBallOne, true);
                    pathState = 2;
                } else if (obeliskReadTimer.milliseconds() > OBELISK_READ_TIMEOUT_MS) {
                    detectedMotifId = 22;
                    panelsTelemetry.debug("no motif detected");
                    follower.followPath(paths.shootBallOne, true);
                    pathState = 2;
                }
                break;

            case 2: // Move to first shoot position (flywheel already spinning from start())
                if (!follower.isBusy()) {
                    prepareForShooting();
                    pathState = 3;
                }
                break;

            case 3: // Shoot 3 preloaded balls
                if (shootBalls()) {
                    follower.followPath(paths.RunToRowOne, true);
                    pathState = 4;
                }
                break;

            case 4: // Run to row 1 intake position
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.intakeRowOne, INTAKE_SPEED, true);
                    pathState = 5;
                }
                break;

            case 5: // Intake row 1 (slow) - pre-spin flywheel during intake
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
                    pathState = 6;
                }
                break;

            case 6: // Move to shoot position + shoot row 1 balls
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                // Only shoot once path completes and robot is in position
                if (!follower.isBusy() && shootBalls()) {
                    shootingPrepared = false;
                    follower.followPath(paths.RuntoRowTwo, true);
                    pathState = 7;
                }
                break;

            case 7: // Run to row 2 intake position
                if (!follower.isBusy()) {
                    prepareForIntake();
                    follower.followPath(paths.intakeRowTwo, INTAKE_SPEED, true);
                    pathState = 8;
                }
                break;

            case 8: // Intake row 2 (slow) - pre-spin flywheel during intake
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
                    pathState = 9;
                }
                break;

            case 9: // Move to shoot position + shoot row 2 balls
                if (!shootingPrepared) {
                    prepareForShooting();
                    shootingPrepared = true;
                }
                if (!follower.isBusy() && shootBalls()) {
                    shootingPrepared = false;
                    follower.followPath(paths.LeavePoints, true);
                    pathState = 10;
                }
                break;

            case 10: // Leave for points
                if (!follower.isBusy()) {
                    pathState = 11;
                }
                break;

            case 11: // Done
                outtake.setRPM(0);
                intake.setPower(0);
                kicker.down();
                limelight.stop();
                requestOpModeStop();
                break;

            default:
                outtake.setRPM(0);
                intake.setPower(0);
                break;
        }
    }
}
