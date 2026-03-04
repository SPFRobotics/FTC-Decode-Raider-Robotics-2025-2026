/* ============================================================= *
 *        Pedro Pathing Plus Visualizer — Auto-Generated         *
 *                                                               *
 *  Version: 1.7.5.                                              *
 *  Copyright (c) 2026 Matthew Allen                             *
 *                                                               *
 *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
 *  Changes will be overwritten when regenerated.                *
 * ============================================================= */

package org.firstinspires.ftc.teamcode.Game.Auto.PedroPaths;

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
import org.firstinspires.ftc.teamcode.Assets.PedroPathing.Constants;

@Autonomous(name = "BlueShortClear", group = "Autonomous")
@Configurable // Panels
public class BlueShort9BallClear extends OpMode {

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
    follower.setStartingPose(
      new Pose(33.400, 133.900, Math.toRadians(180.000))
    );

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

  public static class Paths {

    public PathChain RunToShootPreload;
    public PathChain RunToSpikeTwo;
    public PathChain IntakeSpikeTwo;
    public PathChain RunToShootSpikeTwo;
    public PathChain RunToSpikeOne;
    public PathChain IntakeSpikeOne;
    public PathChain ClearRamp;
    public PathChain RunToShootSpikeOne;
    public PathChain line9;

    public Paths(Follower follower) {
      RunToShootPreload = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(33.400, 133.900), new Pose(47.776, 95.823))
        )
        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .build();

      RunToSpikeTwo = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(47.776, 95.823), new Pose(47.776, 60.164))
        )
        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .build();

      IntakeSpikeTwo = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(47.776, 60.164), new Pose(26.262, 60.164))
        )
        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .build();

      RunToShootSpikeTwo = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(26.262, 60.164), new Pose(49.064, 95.823))
        )
        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .build();

      RunToSpikeOne = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(49.064, 95.823), new Pose(33.400, 84.108))
        )
        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .build();

      IntakeSpikeOne = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(33.400, 84.108), new Pose(25.086, 84.108))
        )
        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
        .build();

      ClearRamp = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(25.086, 84.108), new Pose(16.989, 77.154))
        )
        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
        .build();

      RunToShootSpikeOne = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(16.989, 77.154), new Pose(47.776, 95.823))
        )
        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
        .build();

      line9 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(47.776, 95.823), new Pose(23.000, 74.000))
        )
        .setTangentHeadingInterpolation()
        .build();
    }
  }

  public int autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        follower.followPath(paths.RunToShootPreload, true);
        setPathState(1);
        break;
      case 1:
        if (!follower.isBusy()) {
          setPathState(2);
        }
        break;
      case 2:
        follower.followPath(paths.RunToSpikeTwo, true);
        setPathState(3);
        break;
      case 3:
        if (!follower.isBusy()) {
          setPathState(4);
        }
        break;
      case 4:
        follower.followPath(paths.IntakeSpikeTwo, true);
        setPathState(5);
        break;
      case 5:
        if (!follower.isBusy()) {
          setPathState(6);
        }
        break;
      case 6:
        follower.followPath(paths.RunToShootSpikeTwo, true);
        setPathState(7);
        break;
      case 7:
        if (!follower.isBusy()) {
          setPathState(8);
        }
        break;
      case 8:
        follower.followPath(paths.RunToSpikeOne, true);
        setPathState(9);
        break;
      case 9:
        if (!follower.isBusy()) {
          setPathState(10);
        }
        break;
      case 10:
        follower.followPath(paths.IntakeSpikeOne, true);
        setPathState(11);
        break;
      case 11:
        if (!follower.isBusy()) {
          setPathState(12);
        }
        break;
      case 12:
        follower.turnTo(1.571);
        setPathState(13);
        break;
      case 13:
        if (!follower.isBusy()) {
          setPathState(14);
        }
        break;
      case 14:
        follower.followPath(paths.ClearRamp, true);
        setPathState(15);
        break;
      case 15:
        if (!follower.isBusy()) {
          setPathState(16);
        }
        break;
      case 16:
        setPathState(17);
        break;
      case 17:
        if (pathTimer.milliseconds() > 500) {
          setPathState(18);
        }
        break;
      case 18:
        follower.followPath(paths.RunToShootSpikeOne, true);
        setPathState(19);
        break;
      case 19:
        if (!follower.isBusy()) {
          setPathState(20);
        }
        break;
      case 20:
        follower.followPath(paths.line9, true);
        setPathState(21);
        break;
      case 21:
        if (!follower.isBusy()) {
          setPathState(22);
        }
        break;
      case 22:
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
