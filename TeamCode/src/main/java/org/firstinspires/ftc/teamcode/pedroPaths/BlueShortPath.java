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

import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFinder;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.intakePos;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.outtakePos;

@Autonomous(name = "Blue Short", group = "Autonomous")
@Configurable // Panels
public class BlueShortPath extends OpMode {

    private static final int RUN_FROM_WALL = 0;
    private static final int SHOOT_PRELOAD = 1;  // NEW: Shoot preloaded balls first
    private static final int RUN_TO_FIRST = 2;
    private static final int INTAKE_FIRST = 3;
    private static final int BACK_TO_SHOOT_FIRST = 4;
    private static final int SHOOT_FIRST = 5;
    private static final int RUN_TO_SECOND = 6;
    private static final int INTAKE_SECOND = 7;
    private static final int BACK_TO_SHOOT_SECOND = 8;
    private static final int SHOOT_SECOND = 9;
    private static final int RUN_TO_THIRD = 10;
    private static final int INTAKE_THIRD = 11;
    private static final int BACK_TO_SHOOT_THIRD = 12;
    private static final int SHOOT_THIRD = 13;
    private static final int LEAVE = 14;
    private static final int DONE = 15;

    private static final double SHOOT_RPM = 3200;
    private static final double INTAKE_DISTANCE_CM = 5.7;
    private static final double RELEASE_DISTANCE_CM = 7;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = DONE; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Intake intake;
    private Outtake outtake;
    private Spindex spindex;
    private KickerSpindex kicker;
    private int spindexSlot = 0;
    private ColorFinder colorSensor;
    private int ballCount = 0;
    private boolean ballLatched = false;
    private int lastCycleCount = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        colorSensor = new ColorFinder(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        // Start where the first generated path begins so the follower does not jump
        follower.setStartingPose(new Pose(34.3, 135.0, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        outtake.resetKickerCycle();
        // Preloaded balls: start with 3 balls already in spindex
        ballCount = 3;
        spindexSlot = 0;
        lastCycleCount = 0;
        ballLatched = false;
        spindex.setIndex(spindexSlot);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        checkIntakeAndAdvanceSpindex();
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        updateSpindexTarget();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("BallCount", ballCount);
        panelsTelemetry.debug("SpindexSlot", spindexSlot);
        panelsTelemetry.debug("KickerCycles", outtake.getKickerCycleCount());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public final PathChain runFromWall;
        public final PathChain runToFirstIntakePos;
        public final PathChain intakeFirst;
        public final PathChain goBackToShootingFirst;
        public final PathChain runToSecondIntakePos;
        public final PathChain intakeSecond;
        public final PathChain goBackToShootingSecond;
        public final PathChain runToThirdIntakePos;
        public final PathChain intakeThird;
        public final PathChain goBackToShootingThird;
        public final PathChain leave;

        public Paths(Follower follower) {
            runFromWall = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(34.300, 135.000), new Pose(73.000, 76.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(142))
                    .build();

            runToFirstIntakePos = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(73.000, 76.000), new Pose(47.500, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                    .build();

            intakeFirst = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.500, 85.000), new Pose(16.600, 85.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            goBackToShootingFirst = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.600, 85.000), new Pose(73.000, 76.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();

            runToSecondIntakePos = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(73.000, 76.000), new Pose(47.500, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                    .build();

            intakeSecond = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.500, 60.000), new Pose(15.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingSecond = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 60.000), new Pose(73.000, 76.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();

            runToThirdIntakePos = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(73.000, 76.000), new Pose(47.500, 35.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                    .build();

            intakeThird = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.500, 35.000), new Pose(13.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingThird = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.000, 35.000), new Pose(73.000, 76.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();

            leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(73.000, 76.000), new Pose(55.000, 55.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(142))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Simple sequential state machine: move to the next path once the current one finishes.
        if (pathState == DONE) {
            // First loop after init: kick off the run-from-wall path.
            outtake.setRPM(SHOOT_RPM); // Start spinning up while driving
            follower.followPath(paths.runFromWall);
            return RUN_FROM_WALL;
        }

        int currentCycles = outtake.getKickerCycleCount();

        if (!follower.isBusy()) {
            switch (pathState) {
                case RUN_FROM_WALL:
                    // Arrived at shooting position - shoot preloaded balls
                    outtake.resetKickerCycle();
                    lastCycleCount = 0;
                    spindexSlot = 0;
                    pathState = SHOOT_PRELOAD;
                    break;

                case SHOOT_PRELOAD:
                    // Shoot the 3 preloaded balls
                    outtake.setRPM(SHOOT_RPM);
                    double targetAngle = outtakePos[spindexSlot];
                    double error = Math.abs(AngleUnit.normalizeDegrees(targetAngle - spindex.getPos()));
                    boolean aligned = error <= Spindex.SpindexValues.tolorence;

                    if (aligned) {
                        outtake.enableSpindexKickerCycle(true, SHOOT_RPM);
                    }

                    if (currentCycles > lastCycleCount) {
                        lastCycleCount = currentCycles;
                        spindexSlot = (spindexSlot + 1) % 3;
                    }

                    if (outtake.getKickerCycleCount() >= 3) {
                        outtake.setRPM(0);
                        kicker.down();
                        outtake.resetKickerCycle();
                        lastCycleCount = 0;
                        ballCount = 0;
                        spindexSlot = 0;
                        ballLatched = false;
                        follower.followPath(paths.runToFirstIntakePos);
                        pathState = RUN_TO_FIRST;
                    }
                    break;

                case RUN_TO_FIRST:
                    intake.intakeOn();
                    follower.followPath(paths.intakeFirst);
                    pathState = INTAKE_FIRST;
                    break;
                case INTAKE_FIRST:
                    intake.intakeOff();
                    outtake.setRPM(SHOOT_RPM);
                    follower.followPath(paths.goBackToShootingFirst);
                    pathState = BACK_TO_SHOOT_FIRST;
                    break;
                case BACK_TO_SHOOT_FIRST:
                    outtake.resetKickerCycle();
                    lastCycleCount = 0;
                    pathState = SHOOT_FIRST;
                    break;
                case SHOOT_FIRST:
                    outtake.setRPM(SHOOT_RPM);
                    targetAngle = outtakePos[spindexSlot];
                    error = Math.abs(AngleUnit.normalizeDegrees(targetAngle - spindex.getPos()));
                    aligned = error <= Spindex.SpindexValues.tolorence;

                    if (aligned) {
                        outtake.enableSpindexKickerCycle(true, SHOOT_RPM);
                    }

                    if (currentCycles > lastCycleCount) {
                        lastCycleCount = currentCycles;
                        spindexSlot = (spindexSlot + 1) % 3;
                    }

                    if (outtake.getKickerCycleCount() >= 3) {
                        outtake.setRPM(0);
                        kicker.down();
                        outtake.resetKickerCycle();
                        lastCycleCount = 0;
                        ballCount = 0;
                        spindexSlot = 0;
                        ballLatched = false;
                        follower.followPath(paths.runToSecondIntakePos);
                        pathState = RUN_TO_SECOND;
                    }
                    break;
                case RUN_TO_SECOND:
                    intake.intakeOn();
                    follower.followPath(paths.intakeSecond);
                    pathState = INTAKE_SECOND;
                    break;
                case INTAKE_SECOND:
                    intake.intakeOff();
                    outtake.setRPM(SHOOT_RPM);
                    follower.followPath(paths.goBackToShootingSecond);
                    pathState = BACK_TO_SHOOT_SECOND;
                    break;
                case BACK_TO_SHOOT_SECOND:
                    outtake.resetKickerCycle();
                    lastCycleCount = 0;
                    pathState = SHOOT_SECOND;
                    break;
                case SHOOT_SECOND:
                    outtake.setRPM(SHOOT_RPM);
                    targetAngle = outtakePos[spindexSlot];
                    error = Math.abs(AngleUnit.normalizeDegrees(targetAngle - spindex.getPos()));
                    aligned = error <= Spindex.SpindexValues.tolorence;

                    if (aligned) {
                        outtake.enableSpindexKickerCycle(true, SHOOT_RPM);
                    }

                    if (currentCycles > lastCycleCount) {
                        lastCycleCount = currentCycles;
                        spindexSlot = (spindexSlot + 1) % 3;
                    }

                    if (outtake.getKickerCycleCount() >= 3) {
                        outtake.setRPM(0);
                        kicker.down();
                        outtake.resetKickerCycle();
                        lastCycleCount = 0;
                        ballCount = 0;
                        spindexSlot = 0;
                        ballLatched = false;
                        follower.followPath(paths.runToThirdIntakePos);
                        pathState = RUN_TO_THIRD;
                    }
                    break;
                case RUN_TO_THIRD:
                    intake.intakeOn();
                    follower.followPath(paths.intakeThird);
                    pathState = INTAKE_THIRD;
                    break;
                case INTAKE_THIRD:
                    intake.intakeOff();
                    outtake.setRPM(SHOOT_RPM);
                    follower.followPath(paths.goBackToShootingThird);
                    pathState = BACK_TO_SHOOT_THIRD;
                    break;
                case BACK_TO_SHOOT_THIRD:
                    outtake.resetKickerCycle();
                    lastCycleCount = 0;
                    pathState = SHOOT_THIRD;
                    break;
                case SHOOT_THIRD:
                    outtake.setRPM(SHOOT_RPM);
                    targetAngle = outtakePos[spindexSlot];
                    error = Math.abs(AngleUnit.normalizeDegrees(targetAngle - spindex.getPos()));
                    aligned = error <= Spindex.SpindexValues.tolorence;

                    if (aligned) {
                        outtake.enableSpindexKickerCycle(true, SHOOT_RPM);
                    }

                    if (currentCycles > lastCycleCount) {
                        lastCycleCount = currentCycles;
                        spindexSlot = (spindexSlot + 1) % 3;
                    }

                    if (outtake.getKickerCycleCount() >= 3) {
                        outtake.setRPM(0);
                        kicker.down();
                        outtake.resetKickerCycle();
                        follower.followPath(paths.leave);
                        pathState = LEAVE;
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
            }
        }
        return pathState;
    }

    // Position the spindex based on the current autonomous state and slot.
    private void updateSpindexTarget() {
        if (spindex == null) return;

        switch (pathState) {
            case INTAKE_FIRST:
            case INTAKE_SECOND:
            case INTAKE_THIRD:
                spindex.setIndex(ballCount % 3);
                spindex.moveToPos(intakePos[spindex.getIndex()], true);
                break;
            case RUN_FROM_WALL:  // Position for preload shooting while driving
            case SHOOT_PRELOAD:
            case BACK_TO_SHOOT_FIRST:
            case SHOOT_FIRST:
            case BACK_TO_SHOOT_SECOND:
            case SHOOT_SECOND:
            case BACK_TO_SHOOT_THIRD:
            case SHOOT_THIRD:
                spindex.setIndex(spindexSlot);
                spindex.moveToPos(outtakePos[spindex.getIndex()], true);
                break;
            default:
                // Hold current slot/position; no move call needed.
                break;
        }
    }

    // Match SpindexTestPath behavior: use ball latching for proper detection
    private void checkIntakeAndAdvanceSpindex() {
        if (spindex == null || colorSensor == null) return;

        boolean inIntakeState = pathState == INTAKE_FIRST || pathState == INTAKE_SECOND || pathState == INTAKE_THIRD;
        if (!inIntakeState) return;

        double distance = colorSensor.getDistance();

        // Ball detection with latching (like SpindexTestPath)
        if (distance <= INTAKE_DISTANCE_CM && !ballLatched && ballCount < 3 && spindex.getPower() == 0) {
            ballLatched = true;
            ballCount++;
            spindex.addIndex();
        } else if (distance > RELEASE_DISTANCE_CM) {
            ballLatched = false;
        }
    }
}
