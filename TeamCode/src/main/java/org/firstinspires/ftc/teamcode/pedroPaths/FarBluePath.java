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

import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues;

@Autonomous(name = "Far Blue", group = "Autonomous")
@Configurable
public class FarBluePath extends OpMode {

    private static final int RUN_TO_FIRST = 0;
    private static final int INTAKE_FIRST = 1;
    private static final int BACK_TO_SHOOT_FIRST = 2;
    private static final int TURN_TO_SHOOT_FIRST = 3;
    private static final int SHOOT_FIRST = 4;

    private static final int RUN_TO_SECOND = 5;
    private static final int INTAKE_SECOND = 6;
    private static final int BACK_TO_SHOOT_SECOND = 7;
    private static final int SHOOT_SECOND = 8;

    private static final int RUN_TO_THIRD = 9;
    private static final int INTAKE_THIRD = 10;
    private static final int BACK_TO_SHOOT_THIRD = 11;
    private static final int SHOOT_THIRD = 12;

    private static final int LEAVE = 13;
    private static final int DONE = 14;

    private static final double SHOOT_RPM = 3200;

    private static final boolean USE_ABS_ENCODER = true;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = DONE;
    private Paths paths;

    private Intake intake;
    private Outtake outtake;
    private Spindex spindex;

    // target angle in degrees for the spindex
    private double spindexTargetDeg = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        spindex = new Spindex(hardwareMap);

        // start on first slot in intake mode
        spindex.setIndex(0);
        spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60.000, 10.000, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();

        // always drive the spindex toward its current target
        spindex.moveToPos(spindexTargetDeg, USE_ABS_ENCODER);

        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
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

        public final PathChain runToFirstIntakePos;
        public PathChain intakeFirst = null;
        public final PathChain goBackToShootingFirst;
        public final PathChain turnToShootFirst;

        public final PathChain runToSecondIntakePos;
        public final PathChain intakeSecond;
        public final PathChain goBackToShootingSecond;

        public final PathChain runToThirdIntakePos;
        public final PathChain intakeThird;
        public final PathChain goBackToShootingThird;

        public final PathChain leave;

        public Paths(Follower follower) {

            runToFirstIntakePos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(45.000, 36.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(45.000, 36.000), new Pose(15.000, 36.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingFirst = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.000, 36.000), new Pose(60.000, 10.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            turnToShootFirst = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(60.000, 10.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(110))
                    .build();

            runToSecondIntakePos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(41.849, 59.597)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(41.849, 59.597), new Pose(20.219, 59.361)))
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingSecond = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.219, 59.361), new Pose(60.000, 10.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(109))
                    .build();

            runToThirdIntakePos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(42.554, 83.930)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intakeThird = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(42.554, 83.930), new Pose(18.691, 83.812)))
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingThird = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.691, 83.812), new Pose(60.000, 10.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(110))
                    .build();

            leave = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(55.132, 54.424)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    private boolean spindexAligned() {
        return Math.abs(spindex.getError()) <= SpindexValues.tolorence;
    }

    public int autonomousPathUpdate() {

        if (pathState == DONE) {
            // go to first intake position and aim the spindex at the first intake slot
            spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];
            follower.followPath(paths.runToFirstIntakePos);
            return RUN_TO_FIRST;
        }

        if (!follower.isBusy()) {
            switch (pathState) {

                case RUN_TO_FIRST:
                    intake.intakeOn();
                    spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];
                    follower.followPath(paths.intakeFirst);
                    pathState = INTAKE_FIRST;
                    break;

                case INTAKE_FIRST:
                    intake.intakeOff();
                    follower.followPath(paths.goBackToShootingFirst);
                    pathState = BACK_TO_SHOOT_FIRST;
                    break;

                case BACK_TO_SHOOT_FIRST:
                    follower.followPath(paths.turnToShootFirst);
                    pathState = TURN_TO_SHOOT_FIRST;
                    break;

                case TURN_TO_SHOOT_FIRST:
                    // align spindex to outtake position for current slot
                    spindexTargetDeg = SpindexValues.outtakePos[spindex.getIndex()];
                    outtake.resetKickerCycle();
                    pathState = SHOOT_FIRST;
                    break;

                case SHOOT_FIRST:
                    // wait for spindex to be lined up before actually shooting
                    if (!spindexAligned()) {
                        outtake.setRPM(0);
                        outtake.enableKickerCycle(false, SHOOT_RPM);
                    } else {
                        outtake.setRPM(SHOOT_RPM);
                        outtake.enableKickerCycle(true, SHOOT_RPM);

                        if (outtake.getKickerCycleCount() >= 3) {
                            outtake.setRPM(0);
                            outtake.resetKickerCycle();

                            spindex.clearSlot(spindex.getIndex());
                            spindex.addIndex();

                            follower.followPath(paths.runToSecondIntakePos);
                            // aim spindex to next intake slot while driving
                            spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];
                            pathState = RUN_TO_SECOND;
                        }
                    }
                    break;

                case RUN_TO_SECOND:
                    intake.intakeOn();
                    spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];
                    follower.followPath(paths.intakeSecond);
                    pathState = INTAKE_SECOND;
                    break;

                case INTAKE_SECOND:
                    intake.intakeOff();
                    follower.followPath(paths.goBackToShootingSecond);
                    pathState = BACK_TO_SHOOT_SECOND;
                    break;

                case BACK_TO_SHOOT_SECOND:
                    outtake.resetKickerCycle();
                    // align spindex to outtake position for this slot
                    spindexTargetDeg = SpindexValues.outtakePos[spindex.getIndex()];
                    pathState = SHOOT_SECOND;
                    break;

                case SHOOT_SECOND:
                    if (!spindexAligned()) {
                        outtake.setRPM(0);
                        outtake.enableKickerCycle(false, SHOOT_RPM);
                    } else {
                        outtake.setRPM(SHOOT_RPM);
                        outtake.enableKickerCycle(true, SHOOT_RPM);

                        if (outtake.getKickerCycleCount() >= 3) {
                            outtake.setRPM(0);
                            outtake.resetKickerCycle();

                            spindex.clearSlot(spindex.getIndex());
                            spindex.addIndex();

                            follower.followPath(paths.runToThirdIntakePos);
                            spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];
                            pathState = RUN_TO_THIRD;
                        }
                    }
                    break;

                case RUN_TO_THIRD:
                    intake.intakeOn();
                    spindexTargetDeg = SpindexValues.intakePos[spindex.getIndex()];
                    follower.followPath(paths.intakeThird);
                    pathState = INTAKE_THIRD;
                    break;

                case INTAKE_THIRD:
                    intake.intakeOff();
                    follower.followPath(paths.goBackToShootingThird);
                    pathState = BACK_TO_SHOOT_THIRD;
                    break;

                case BACK_TO_SHOOT_THIRD:
                    outtake.resetKickerCycle();
                    spindexTargetDeg = SpindexValues.outtakePos[spindex.getIndex()];
                    pathState = SHOOT_THIRD;
                    break;

                case SHOOT_THIRD:
                    if (!spindexAligned()) {
                        outtake.setRPM(0);
                        outtake.enableKickerCycle(false, SHOOT_RPM);
                    } else {
                        outtake.setRPM(SHOOT_RPM);
                        outtake.enableKickerCycle(true, SHOOT_RPM);

                        if (outtake.getKickerCycleCount() >= 3) {
                            outtake.setRPM(0);
                            outtake.resetKickerCycle();

                            spindex.clearSlot(spindex.getIndex());
                            spindex.addIndex();

                            follower.followPath(paths.leave);
                            pathState = LEAVE;
                        }
                    }
                    break;

                case LEAVE:
                    intake.intakeOff();
                    outtake.setRPM(0);
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
