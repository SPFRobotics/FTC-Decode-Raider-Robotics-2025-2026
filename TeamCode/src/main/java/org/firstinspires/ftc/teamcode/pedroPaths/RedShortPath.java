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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;

@Autonomous(name = "Red Short Path", group = "Autonomous")
@Configurable // Panels
public class RedShortPath extends OpMode {

    private static final int RUN_FROM_WALL = 0;
    private static final int RUN_TO_FIRST = 1;
    private static final int INTAKE_FIRST = 2;
    private static final int BACK_TO_SHOOT_FIRST = 3;
    private static final int RUN_TO_SECOND = 4;
    private static final int INTAKE_SECOND = 5;
    private static final int BACK_TO_SHOOT_SECOND = 6;
    private static final int RUN_TO_THIRD = 7;
    private static final int INTAKE_THIRD = 8;
    private static final int BACK_TO_SHOOT_THIRD = 9;
    private static final int LEAVE = 10;
    private static final int DONE = 11;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = DONE; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Intake intake;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        // Align with first path start to avoid pose jump
        follower.setStartingPose(new Pose(110.1, 134.8, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
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
                            new BezierLine(new Pose(110.100, 134.800), new Pose(73.000, 76.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(41))
                    .build();

            runToFirstIntakePos = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(73.000, 76.000), new Pose(98.900, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(360))
                    .build();

            intakeFirst = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(98.900, 85.000), new Pose(129.600, 85.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(360))
                    .build();

            goBackToShootingFirst = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(129.600, 85.000), new Pose(71.000, 76.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(41))
                    .build();

            runToSecondIntakePos = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(71.000, 76.000), new Pose(98.900, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(360))
                    .build();

            intakeSecond = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(98.900, 60.000), new Pose(129.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingSecond = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(129.000, 60.000), new Pose(71.000, 76.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(41))
                    .build();

            runToThirdIntakePos = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(71.000, 76.000), new Pose(98.900, 35.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(360))
                    .build();

            intakeThird = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(98.900, 35.000), new Pose(131.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingThird = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(131.000, 35.000), new Pose(71.000, 76.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(41))
                    .build();

            leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(71.000, 76.000), new Pose(89.000, 55.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(41))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Simple sequential state machine: advance when the current path finishes.
        if (pathState == DONE) {
            follower.followPath(paths.runFromWall);
            return RUN_FROM_WALL;
        }

        if (!follower.isBusy()) {
            switch (pathState) {
                case RUN_FROM_WALL:
                    follower.followPath(paths.runToFirstIntakePos);
                    pathState = RUN_TO_FIRST;
                    break;
                case RUN_TO_FIRST:
                    intake.intakeOn();
                    follower.followPath(paths.intakeFirst);
                    pathState = INTAKE_FIRST;
                    break;
                case INTAKE_FIRST:
                    intake.intakeOff();
                    follower.followPath(paths.goBackToShootingFirst);
                    pathState = BACK_TO_SHOOT_FIRST;
                    break;
                case BACK_TO_SHOOT_FIRST:
                    follower.followPath(paths.runToSecondIntakePos);
                    pathState = RUN_TO_SECOND;
                    break;
                case RUN_TO_SECOND:
                    intake.intakeOn();
                    follower.followPath(paths.intakeSecond);
                    pathState = INTAKE_SECOND;
                    break;
                case INTAKE_SECOND:
                    intake.intakeOff();
                    follower.followPath(paths.goBackToShootingSecond);
                    pathState = BACK_TO_SHOOT_SECOND;
                    break;
                case BACK_TO_SHOOT_SECOND:
                    follower.followPath(paths.runToThirdIntakePos);
                    pathState = RUN_TO_THIRD;
                    break;
                case RUN_TO_THIRD:
                    intake.intakeOn();
                    follower.followPath(paths.intakeThird);
                    pathState = INTAKE_THIRD;
                    break;
                case INTAKE_THIRD:
                    intake.intakeOff();
                    follower.followPath(paths.goBackToShootingThird);
                    pathState = BACK_TO_SHOOT_THIRD;
                    break;
                case BACK_TO_SHOOT_THIRD:
                    intake.intakeOff();
                    follower.followPath(paths.leave);
                    pathState = LEAVE;
                    break;
                case LEAVE:
                    intake.intakeOff();
                    pathState = DONE;
                    requestOpModeStop();
                    break;
                default:
                    pathState = DONE;
            }
        }
        return pathState;
    }
}