package org.firstinspires.ftc.teamcode.pedroPaths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Path Test 2", group = "Autonomous")
@Configurable // Panels
public class PathTest2 extends OpMode {

    // State machine constants
    private static final int IDLE = 0;
    private static final int PATH_1 = 1;
    private static final int PATH_2 = 2;
    private static final int DONE = 3;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = IDLE; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Start at the beginning of Path1
        follower.setStartingPose(new Pose(40.226, 135.645, Math.toRadians(90)));

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
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(40.226, 135.645),
                                    new Pose(70.774, 74.129)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(70.774, 74.129),
                                    new Pose(40.323, 84.355)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // State machine: execute paths sequentially
        switch (pathState) {
            case IDLE:
                // Start following the first path
                follower.followPath(paths.Path1);
                return PATH_1;

            case PATH_1:
                // Wait for Path1 to complete, then start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    return PATH_2;
                }
                return PATH_1;

            case PATH_2:
                // Wait for Path2 to complete
                if (!follower.isBusy()) {
                    return DONE;
                }
                return PATH_2;

            case DONE:
                // Autonomous complete
                requestOpModeStop();
                return DONE;

            default:
                return IDLE;
        }
    }
}