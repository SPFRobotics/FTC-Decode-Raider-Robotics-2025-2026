package org.firstinspires.ftc.teamcode.pedroPaths;

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

import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues;

@Autonomous(name = "Far Blue Path", group = "Autonomous")
@Configurable
public class FarBluePath extends OpMode {

    private static final int TURN_TO_PRELOAD_SHOOT = 0;
    private static final int SHOOT_PRELOAD = 1;

    private static final int RUN_TO_FIRST_A = 2;
    private static final int RUN_TO_FIRST_B = 3;
    private static final int INTAKE_FIRST = 4;
    private static final int BACK_TO_SHOOT_FIRST = 5;
    private static final int TURN_TO_SHOOT_FIRST = 6;
    private static final int SHOOT_FIRST = 7;

    private static final int RUN_TO_SECOND_A = 8;
    private static final int RUN_TO_SECOND_B = 9;
    private static final int INTAKE_SECOND = 10;
    private static final int BACK_TO_SHOOT_SECOND = 11;
    private static final int TURN_TO_SHOOT_SECOND = 12;
    private static final int SHOOT_SECOND = 13;

    private static final int RUN_TO_THIRD_A = 14;
    private static final int RUN_TO_THIRD_B = 15;
    private static final int INTAKE_THIRD = 16;
    private static final int BACK_TO_SHOOT_THIRD = 17;
    private static final int TURN_TO_SHOOT_THIRD = 18;
    private static final int SHOOT_THIRD = 19;

    private static final int LEAVE = 20;
    private static final int DONE = 21;

    private static final double SHOOT_RPM = 3200;
    private static final boolean USE_ABS_ENCODER = true;

    private static final double INTAKE_TIMEOUT_MS = 2600;
    private static final double BETWEEN_PATH_PAUSE_MS = 60;

    private TelemetryManager panelsTelemetry;

    public Follower follower;
    private Paths paths;

    private Intake intake;
    private Outtake outtake;
    private Spindex spindex;
    private ColorFetch colorSensor;

    private boolean outtakeEnabled = true;
    private int pathState = DONE;

    private double spindexTargetDeg = 0.0;

    private final ElapsedTime stepClock = new ElapsedTime();

    private boolean intakeBallLatched = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap);

        try {
            outtake = new Outtake(hardwareMap);
            outtakeEnabled = true;
        } catch (Exception e) {
            outtake = null;
            outtakeEnabled = false;
        }

        spindex = new Spindex(hardwareMap);
        colorSensor = new ColorFetch(hardwareMap);

        spindex.setIndex(0);
        spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(61.000, 8.000, Math.toRadians(90)));

        paths = new Paths(follower);

        stepClock.reset();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Outtake Enabled", outtakeEnabled);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        stepClock.reset();
        follower.followPath(paths.Path1);
        pathState = TURN_TO_PRELOAD_SHOOT;
    }

    @Override
    public void loop() {
        follower.update();

        spindex.moveToPos(spindexTargetDeg, USE_ABS_ENCODER);

        runAutoLoadWhileIntaking();

        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Outtake Enabled", outtakeEnabled);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Spindex Index", spindex.getIndex());
        panelsTelemetry.debug("Spindex Target", spindexTargetDeg);
        panelsTelemetry.debug("Spindex Pos", spindex.getPos());
        panelsTelemetry.debug("Spindex Error", spindex.getError());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.000, 8.000), new Pose(61.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.000, 8.000), new Pose(61.000, 8.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(110))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.000, 8.000), new Pose(47.543, 35.366)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(47.543, 35.366), new Pose(19.952, 35.144)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(19.952, 35.144), new Pose(61.000, 8.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.000, 8.000), new Pose(61.000, 8.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(110))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.000, 8.000), new Pose(47.976, 59.722)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(47.976, 59.722), new Pose(17.038, 59.885)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(17.038, 59.885), new Pose(61.000, 8.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.000, 8.000), new Pose(61.000, 8.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(110))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.000, 8.000), new Pose(46.593, 83.909)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(46.593, 83.909), new Pose(15.522, 84.287)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.522, 84.287), new Pose(61.000, 8.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path14 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.000, 8.000), new Pose(61.000, 8.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(110))
                    .build();

            Path15 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(61.000, 8.000), new Pose(60.493, 37.785)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    private boolean spindexAligned() {
        return Math.abs(spindex.getError()) <= SpindexValues.tolorence;
    }

    private void safeSetRPM(double rpm) {
        if (!outtakeEnabled || outtake == null) return;
        outtake.setRPM(rpm);
    }

    private void safeResetKicker() {
        if (!outtakeEnabled || outtake == null) return;
        outtake.resetKickerCycle();
    }

    private void safeEnableKicker(boolean enabled, double rpm) {
        if (!outtakeEnabled || outtake == null) return;
        outtake.enableKickerCycle(enabled, rpm);
    }

    private int safeKickerCount() {
        if (!outtakeEnabled || outtake == null) return 999;
        return outtake.getKickerCycleCount();
    }

    private void skipShootingTo(int nextState, PathChain nextPath) {
        spindex.clearSlot(spindex.getIndex());
        spindex.clearBall(spindex.getIndex());
        spindex.addIndex();
        if (nextPath != null) follower.followPath(nextPath);
        spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];
        stepClock.reset();
        pathState = nextState;
    }

    private void runAutoLoadWhileIntaking() {
        boolean isIntakingState =
                pathState == RUN_TO_FIRST_A || pathState == RUN_TO_FIRST_B || pathState == INTAKE_FIRST ||
                        pathState == RUN_TO_SECOND_A || pathState == RUN_TO_SECOND_B || pathState == INTAKE_SECOND ||
                        pathState == RUN_TO_THIRD_A || pathState == RUN_TO_THIRD_B || pathState == INTAKE_THIRD;

        spindex.setMode(false);

        if (!isIntakingState) {
            spindex.setAutoLoadMode(false);
            intakeBallLatched = false;
            return;
        }

        spindex.setAutoLoadMode(true);
        spindex.autoLoad(colorSensor);

        boolean hasBallNow = spindex.getSlotStatus()[spindex.getIndex()];
        if (hasBallNow && !intakeBallLatched) {
            intakeBallLatched = true;
            spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];
        }
        if (!hasBallNow) intakeBallLatched = false;

        spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];
    }

    private boolean allSlotsFull() {
        boolean[] s = spindex.getSlotStatus();
        return s[0] && s[1] && s[2];
    }

    private int countLoadedBalls() {
        boolean[] s = spindex.getSlotStatus();
        int c = 0;
        if (s[0]) c++;
        if (s[1]) c++;
        if (s[2]) c++;
        return c;
    }

    private void pauseThen(Runnable r) {
        if (stepClock.milliseconds() >= BETWEEN_PATH_PAUSE_MS) {
            stepClock.reset();
            r.run();
        }
    }

    public int autonomousPathUpdate() {

        if (pathState == DONE) {
            follower.followPath(paths.Path2);
            pathState = TURN_TO_PRELOAD_SHOOT;
            stepClock.reset();
            return pathState;
        }

        if (!follower.isBusy()) {
            switch (pathState) {

                case TURN_TO_PRELOAD_SHOOT:
                    spindexTargetDeg = SpindexValues.outtakePos[spindex.getIndex()];
                    safeResetKicker();
                    pathState = SHOOT_PRELOAD;
                    break;

                case SHOOT_PRELOAD:
                    if (!outtakeEnabled) {
                        follower.followPath(paths.Path3);
                        pathState = RUN_TO_FIRST_A;
                        stepClock.reset();
                        break;
                    }

                    if (!spindexAligned()) {
                        safeSetRPM(0);
                        safeEnableKicker(false, SHOOT_RPM);
                    } else {
                        safeSetRPM(SHOOT_RPM);
                        safeEnableKicker(true, SHOOT_RPM);

                        if (safeKickerCount() >= 3) {
                            safeSetRPM(0);
                            safeResetKicker();

                            spindex.clearSlot(spindex.getIndex());
                            spindex.clearBall(spindex.getIndex());
                            spindex.addIndex();

                            follower.followPath(paths.Path3);
                            pathState = RUN_TO_FIRST_A;
                            stepClock.reset();
                        }
                    }
                    break;

                case RUN_TO_FIRST_A:
                    intake.intakeOn();
                    pauseThen(() -> {
                        follower.followPath(paths.Path4);
                        pathState = RUN_TO_FIRST_B;
                    });
                    break;

                case RUN_TO_FIRST_B:
                    pathState = INTAKE_FIRST;
                    stepClock.reset();
                    break;

                case INTAKE_FIRST:
                    if (allSlotsFull() || stepClock.milliseconds() >= INTAKE_TIMEOUT_MS) {
                        intake.intakeOff();
                        follower.followPath(paths.Path5);
                        pathState = BACK_TO_SHOOT_FIRST;
                        stepClock.reset();
                    }
                    break;

                case BACK_TO_SHOOT_FIRST:
                    follower.followPath(paths.Path6);
                    pathState = TURN_TO_SHOOT_FIRST;
                    stepClock.reset();
                    break;

                case TURN_TO_SHOOT_FIRST:
                    spindexTargetDeg = SpindexValues.outtakePos[spindex.getIndex()];
                    safeResetKicker();
                    pathState = SHOOT_FIRST;
                    break;

                case SHOOT_FIRST:
                    if (!outtakeEnabled) {
                        skipShootingTo(RUN_TO_SECOND_A, paths.Path7);
                        break;
                    }

                    if (!spindexAligned()) {
                        safeSetRPM(0);
                        safeEnableKicker(false, SHOOT_RPM);
                    } else {
                        safeSetRPM(SHOOT_RPM);
                        safeEnableKicker(true, SHOOT_RPM);

                        if (safeKickerCount() >= 3 || countLoadedBalls() == 0) {
                            safeSetRPM(0);
                            safeResetKicker();

                            spindex.clearSlot(spindex.getIndex());
                            spindex.clearBall(spindex.getIndex());
                            spindex.addIndex();

                            follower.followPath(paths.Path7);
                            pathState = RUN_TO_SECOND_A;
                            stepClock.reset();
                        }
                    }
                    break;

                case RUN_TO_SECOND_A:
                    intake.intakeOn();
                    pauseThen(() -> {
                        follower.followPath(paths.Path8);
                        pathState = RUN_TO_SECOND_B;
                    });
                    break;

                case RUN_TO_SECOND_B:
                    pathState = INTAKE_SECOND;
                    stepClock.reset();
                    break;

                case INTAKE_SECOND:
                    if (allSlotsFull() || stepClock.milliseconds() >= INTAKE_TIMEOUT_MS) {
                        intake.intakeOff();
                        follower.followPath(paths.Path9);
                        pathState = BACK_TO_SHOOT_SECOND;
                        stepClock.reset();
                    }
                    break;

                case BACK_TO_SHOOT_SECOND:
                    follower.followPath(paths.Path10);
                    pathState = TURN_TO_SHOOT_SECOND;
                    stepClock.reset();
                    break;

                case TURN_TO_SHOOT_SECOND:
                    spindexTargetDeg = SpindexValues.outtakePos[spindex.getIndex()];
                    safeResetKicker();
                    pathState = SHOOT_SECOND;
                    break;

                case SHOOT_SECOND:
                    if (!outtakeEnabled) {
                        skipShootingTo(RUN_TO_THIRD_A, paths.Path11);
                        break;
                    }

                    if (!spindexAligned()) {
                        safeSetRPM(0);
                        safeEnableKicker(false, SHOOT_RPM);
                    } else {
                        safeSetRPM(SHOOT_RPM);
                        safeEnableKicker(true, SHOOT_RPM);

                        if (safeKickerCount() >= 3 || countLoadedBalls() == 0) {
                            safeSetRPM(0);
                            safeResetKicker();

                            spindex.clearSlot(spindex.getIndex());
                            spindex.clearBall(spindex.getIndex());
                            spindex.addIndex();

                            follower.followPath(paths.Path11);
                            pathState = RUN_TO_THIRD_A;
                            stepClock.reset();
                        }
                    }
                    break;

                case RUN_TO_THIRD_A:
                    intake.intakeOn();
                    pauseThen(() -> {
                        follower.followPath(paths.Path12);
                        pathState = RUN_TO_THIRD_B;
                    });
                    break;

                case RUN_TO_THIRD_B:
                    pathState = INTAKE_THIRD;
                    stepClock.reset();
                    break;

                case INTAKE_THIRD:
                    if (allSlotsFull() || stepClock.milliseconds() >= INTAKE_TIMEOUT_MS) {
                        intake.intakeOff();
                        follower.followPath(paths.Path13);
                        pathState = BACK_TO_SHOOT_THIRD;
                        stepClock.reset();
                    }
                    break;

                case BACK_TO_SHOOT_THIRD:
                    follower.followPath(paths.Path14);
                    pathState = TURN_TO_SHOOT_THIRD;
                    stepClock.reset();
                    break;

                case TURN_TO_SHOOT_THIRD:
                    spindexTargetDeg = SpindexValues.outtakePos[spindex.getIndex()];
                    safeResetKicker();
                    pathState = SHOOT_THIRD;
                    break;

                case SHOOT_THIRD:
                    if (!outtakeEnabled) {
                        follower.followPath(paths.Path15);
                        pathState = LEAVE;
                        stepClock.reset();
                        break;
                    }

                    if (!spindexAligned()) {
                        safeSetRPM(0);
                        safeEnableKicker(false, SHOOT_RPM);
                    } else {
                        safeSetRPM(SHOOT_RPM);
                        safeEnableKicker(true, SHOOT_RPM);

                        if (safeKickerCount() >= 3 || countLoadedBalls() == 0) {
                            safeSetRPM(0);
                            safeResetKicker();

                            spindex.clearSlot(spindex.getIndex());
                            spindex.clearBall(spindex.getIndex());
                            spindex.addIndex();

                            follower.followPath(paths.Path15);
                            pathState = LEAVE;
                            stepClock.reset();
                        }
                    }
                    break;

                case LEAVE:
                    intake.intakeOff();
                    safeSetRPM(0);
                    pathState = DONE;
                    requestOpModeStop();
                    break;

                default:
                    pathState = DONE;
                    break;
            }
        }

        return pathState;
    }
}
