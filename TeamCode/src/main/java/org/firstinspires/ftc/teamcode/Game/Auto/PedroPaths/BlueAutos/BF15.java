package org.firstinspires.ftc.teamcode.Game.Auto.PedroPaths.BlueAutos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Assets.PedroPathing.Constants;

    @Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
    @Configurable // Panels
    public class BF15 extends OpMode {

        private TelemetryManager panelsTelemetry; // Panels Telemetry instance
        public Follower follower; // Pedro Pathing follower instance
        private int pathState; // Current autonomous path state (state machine)
        private ElapsedTime pathTimer; // Timer for path state machine
        private Paths paths; // Paths defined in the Paths class

        @Override
        public void init() {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
            // ...
            panelsTelemetry.debug("Status", "Initialized");
            panelsTelemetry.update(telemetry);

            follower = Constants.createFollower(hardwareMap);
            // Determine starting heading: prefer geometric heading when a path exists, otherwise fall back to explicit startPoint values
            follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(180.000)));

            pathTimer = new ElapsedTime();
            paths = new Paths(follower); // Build paths
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

            public PathChain RunToSpikeOne;
            public PathChain IntakeSpikeOne;
            public PathChain ShootSpikeOne;
            public PathChain RunToHumanSpike;
            public PathChain IntakeHumanSpike;
            public PathChain ContinueHumanSpikeIntake;
            public PathChain ShootHumanSpike;
            public PathChain RunToHailMary;
            public PathChain IntakeHailMary;
            public PathChain ShootHailMary;
            public PathChain RunToHailMary2;
            public PathChain IntakeHailMary2;
            public PathChain ShootHailMary2;
            public PathChain Leave;

            public Paths(Follower follower) {
                RunToSpikeOne = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(56.000, 8.000),
                                        new Pose(51.581, 25.791),
                                        new Pose(41.116, 35.349)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                IntakeSpikeOne = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(41.116, 35.349), new Pose(24.791, 35.674))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                ShootSpikeOne = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(24.791, 35.674), new Pose(56.070, 8.651))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                RunToHumanSpike = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(56.070, 8.651), new Pose(11.200, 33.023))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-170))
                        .build();

                IntakeHumanSpike = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(11.200, 33.023), new Pose(11.200, 10.800))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-170))
                        .build();

                ContinueHumanSpikeIntake = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(11.200, 10.800), new Pose(9.600, 10.800))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-170), Math.toRadians(180))
                        .build();

                ShootHumanSpike = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(9.600, 10.800),
                                        new Pose(30.372, 13.605),
                                        new Pose(56.535, 8.581)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                RunToHailMary = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(56.535, 8.581), new Pose(11.000, 22.279))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                        .build();

                IntakeHailMary = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(11.000, 22.279), new Pose(11.000, 33.256))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build();

                ShootHailMary = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(11.000, 33.256), new Pose(56.302, 8.930))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                        .build();

                RunToHailMary2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(56.302, 8.930), new Pose(10.558, 28.628))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                        .build();

                IntakeHailMary2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(10.558, 28.628), new Pose(10.349, 32.907))
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                ShootHailMary2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(10.349, 32.907), new Pose(57.070, 9.209))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                        .build();

                Leave = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57.070, 9.209), new Pose(10.465, 39.116))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                        .build();
            }
        }


        public int autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    follower.followPath(paths.RunToSpikeOne, true);
                    setPathState(1);
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    follower.followPath(paths.IntakeSpikeOne, true);
                    setPathState(3);
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        setPathState(4);
                    }
                    break;
                case 4:
                    follower.followPath(paths.ShootSpikeOne, true);
                    setPathState(5);
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        setPathState(6);
                    }
                    break;
                case 6:
                    follower.followPath(paths.RunToHumanSpike, true);
                    setPathState(7);
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        setPathState(8);
                    }
                    break;
                case 8:
                    follower.followPath(paths.IntakeHumanSpike, true);
                    setPathState(9);
                    break;
                case 9:
                    if (!follower.isBusy()) {
                        setPathState(10);
                    }
                    break;
                case 10:
                    follower.followPath(paths.ContinueHumanSpikeIntake, true);
                    setPathState(11);
                    break;
                case 11:
                    if (!follower.isBusy()) {
                        setPathState(12);
                    }
                    break;
                case 12:
                    follower.followPath(paths.ShootHumanSpike, true);
                    setPathState(13);
                    break;
                case 13:
                    if (!follower.isBusy()) {
                        setPathState(14);
                    }
                    break;
                case 14:
                    follower.followPath(paths.RunToHailMary, true);
                    setPathState(15);
                    break;
                case 15:
                    if (!follower.isBusy()) {
                        setPathState(16);
                    }
                    break;
                case 16:
                    follower.followPath(paths.IntakeHailMary, true);
                    setPathState(17);
                    break;
                case 17:
                    if (!follower.isBusy()) {
                        setPathState(18);
                    }
                    break;
                case 18:
                    follower.followPath(paths.ShootHailMary, true);
                    setPathState(19);
                    break;
                case 19:
                    if (!follower.isBusy()) {
                        setPathState(20);
                    }
                    break;
                case 20:
                    follower.followPath(paths.RunToHailMary2, true);
                    setPathState(21);
                    break;
                case 21:
                    if (!follower.isBusy()) {
                        setPathState(22);
                    }
                    break;
                case 22:
                    follower.followPath(paths.IntakeHailMary2, true);
                    setPathState(23);
                    break;
                case 23:
                    if (!follower.isBusy()) {
                        setPathState(24);
                    }
                    break;
                case 24:
                    follower.followPath(paths.ShootHailMary2, true);
                    setPathState(25);
                    break;
                case 25:
                    if (!follower.isBusy()) {
                        setPathState(26);
                    }
                    break;
                case 26:
                    follower.followPath(paths.Leave, true);
                    setPathState(27);
                    break;
                case 27:
                    if (!follower.isBusy()) {
                        setPathState(28);
                    }
                    break;
                case 28:
                    requestOpModeStop();
                    pathState = -1;
                    break;
            }
            return pathState;
        }

        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.reset();
        }
    }



