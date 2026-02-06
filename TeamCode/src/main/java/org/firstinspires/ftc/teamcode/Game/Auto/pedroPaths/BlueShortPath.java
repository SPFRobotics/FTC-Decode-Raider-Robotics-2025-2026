package org.firstinspires.ftc.teamcode.Game.Auto.pedroPaths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;

@Autonomous(name = "Blue Short", group = "Autonomous")
@Configurable
public class BlueShortPath extends OpMode {

    private static final double SHOOT_RPM = Outtake.OuttakeConfig.closeRPM;
    private static final double INTAKE_SPEED = 0.3;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private int pathState;

    private Spindex spindex;
    private Outtake outtake;
    private Intake intake;
    private KickerSpindex kicker;
    private ColorFetch colorSensor;

    private int shotsFired = 0;
    private int ballsLoaded = 0;
    private int lastKickerCycles = 0;
    private boolean waitingForSpindexAlign = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(39.000, 134.442, Math.toRadians(90)));
        paths = new Paths(follower);

        spindex = new Spindex(hardwareMap);
        outtake = new Outtake(hardwareMap, true);
        intake = new Intake(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        colorSensor = new ColorFetch(hardwareMap);

        spindex.setAutoLoadMode(true);
        outtake.setRPM(SHOOT_RPM);
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
        follower.followPath(paths.shootBallOne, true);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        updateSpindexPosition();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shots Fired", shotsFired);
        panelsTelemetry.debug("Balls Loaded", ballsLoaded);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("RPM", outtake.getRPM());
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

        double targetAngle = Spindex.SpindexValues.outtakePos[spindex.getIndex()];
        double error = Math.abs(AngleUnit.normalizeDegrees(targetAngle - spindex.getPos()));
        boolean aligned = error <= Spindex.SpindexValues.tolorence;

        // If waiting for spindex to align after advancing
        if (waitingForSpindexAlign) {
            if (aligned) {
                // Spindex reached new position, reset timer and resume shooting
                outtake.resetKickerCycle();
                lastKickerCycles = 0;
                waitingForSpindexAlign = false;
            }
            return false;
        }

        // Only run kicker cycle when aligned
        if (aligned) {
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
                    new BezierCurve(
                            new Pose(39.000, 134.442),
                            new Pose(40.058, 101.070),
                            new Pose(57.674, 86.442)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
            .build();

            RunToRowOne = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(57.674, 86.442),
                            new Pose(53.837, 83.221),
                            new Pose(41.517, 83.829)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
            .build();

            intakeRowOne = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(41.517, 83.829),
                            new Pose(18.000, 84.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

            shootRowOne = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(18.000, 84.000),
                            new Pose(46.806, 84.682),
                            new Pose(47.659, 83.147),
                            new Pose(57.488, 86.140)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
            .build();

            RuntoRowTwo = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(57.488, 86.140),
                            new Pose(60.000, 84.000),
                            new Pose(57.100, 64.389),
                            new Pose(41.346, 61.488)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(185))
            .build();

            intakeRowTwo = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(41.346, 61.488),
                            new Pose(16.000, 58.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(185), Math.toRadians(185))
            .build();

            shootRowTwo = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(16.000, 58.000),
                            new Pose(24.000, 58.000),
                            new Pose(52.095, 64.834),
                            new Pose(57.488, 85.953)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(185), Math.toRadians(135))
            .build();

            RuntwoRowThree = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(57.488, 85.953),
                            new Pose(56.775, 50.205),
                            new Pose(41.142, 35.613)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
            .build();

            IntakeRowThree = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(41.142, 35.613),
                            new Pose(11.445, 34.825)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

            backToShooting = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(11.445, 34.825),
                            new Pose(43.263, 52.457),
                            new Pose(57.479, 86.374)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
            .build();

            LeavePoints = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(57.479, 86.374),
                            new Pose(47.938, 73.403),
                            new Pose(16.209, 69.100)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
            .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to first shoot position
                if (!follower.isBusy()) {
                    prepareForShooting();
                    pathState = 1;
                }
                break;

            case 1: // Shoot 3 preloaded balls
                if (shootBalls()) {
                    prepareForIntake();
                    follower.followPath(paths.RunToRowOne, true);
                    pathState = 2;
                }
                break;

            case 2: // Run to row 1 intake position
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakeRowOne, INTAKE_SPEED, true);
                    pathState = 3;
                }
                break;

            case 3: // Intake row 1 (slow)
                runIntake();
                if (!follower.isBusy()) {
                    prepareForShooting();
                    follower.followPath(paths.shootRowOne, true);
                    pathState = 4;
                }
                break;

            case 4: // Move to shoot position
                if (!follower.isBusy()) {
                    pathState = 5;
                }
                break;

            case 5: // Shoot row 1 balls
                if (shootBalls()) {
                    prepareForIntake();
                    follower.followPath(paths.RuntoRowTwo, true);
                    pathState = 6;
                }
                break;

            case 6: // Run to row 2 intake position
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakeRowTwo, INTAKE_SPEED, true);
                    pathState = 7;
                }
                break;

            case 7: // Intake row 2 (slow)
                runIntake();
                if (!follower.isBusy()) {
                    prepareForShooting();
                    follower.followPath(paths.shootRowTwo, true);
                    pathState = 8;
                }
                break;

            case 8: // Move to shoot position
                if (!follower.isBusy()) {
                    pathState = 9;
                }
                break;

            case 9: // Shoot row 2 balls
                if (shootBalls()) {
                    prepareForIntake();
                    follower.followPath(paths.RuntwoRowThree, true);
                    pathState = 10;
                }
                break;

            case 10: // Run to row 3 intake position
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeRowThree, INTAKE_SPEED, true);
                    pathState = 11;
                }
                break;

            case 11: // Intake row 3 (slow)
                runIntake();
                if (!follower.isBusy()) {
                    prepareForShooting();
                    follower.followPath(paths.backToShooting, true);
                    pathState = 12;
                }
                break;

            case 12: // Move to final shoot position
                if (!follower.isBusy()) {
                    pathState = 13;
                }
                break;

            case 13: // Shoot final 3 balls
                if (shootBalls()) {
                    follower.followPath(paths.LeavePoints, true);
                    pathState = 14;
                }
                break;

            case 14: // Leave for points
                if (!follower.isBusy()) {
                    pathState = 15;
                }
                break;

            case 15: // Done
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
