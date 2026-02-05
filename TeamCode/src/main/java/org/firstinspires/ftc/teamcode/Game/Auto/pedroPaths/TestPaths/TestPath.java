package org.firstinspires.ftc.teamcode.Game.Auto.pedroPaths.TestPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Autonomous(name = "TestPath", group = "Autonomous")

@Disabled
public class TestPath extends OpMode {

    private static final double SHOOT_RPM = 3200;
    private static final int RUN_PATH = 0;
    private static final int SHOOT = 1;
    private static final int DONE = 2;

    private Outtake outtake;
    private Follower follower;
    private PathChain warmupPath;
    private int state = DONE;

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // Start at origin for this simple test
        follower.setStartingPose(new Pose(0.0, 0.0, 0.0));

        // Short path to exercise the Pedro follower; keeps the system "busy" once.
        warmupPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(0.0, 0.0, 0.0),
                        new Pose(-10.0, 0.0, 0.0)))
                .setConstantHeadingInterpolation(0.0)
                .build();

        telemetry.addData("Status", "Ready to run kicker 3x with Pedro");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        if (state == DONE) {
            follower.followPath(warmupPath);
            state = RUN_PATH;
        }

        // When the path finishes, perform the 3-shot cycle.
        if (!follower.isBusy()) {
            switch (state) {
                case RUN_PATH:
                    outtake.resetKickerCycle();
                    outtake.setRPM(SHOOT_RPM);
                    state = SHOOT;
                    break;
                case SHOOT:
                    outtake.enableKickerCycle(true, SHOOT_RPM);
                    if (outtake.getKickerCycleCount() >= 3) {
                        outtake.setRPM(0);
                        outtake.resetKickerCycle();
                        requestOpModeStop();
                        state = DONE;
                    }
                    break;
                default:
                    break;
            }
        }

        telemetry.addData("State", state);
        telemetry.addData("Cycles", outtake.getKickerCycleCount());
        telemetry.addData("RPM", outtake.getRPM());
        telemetry.addData("CycleTime", outtake.getCurrentCycleTime());
        telemetry.update();
    }
}
