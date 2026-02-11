package org.firstinspires.ftc.teamcode.Testing.TestPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Autonomous(name="Pathing Test")
@Disabled
public class PathingTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose STARTPOSE = new Pose(0, 12, Math.toRadians(90));
    private final Pose TURN = new Pose(0, 0, Math.toRadians(90));

    private Path turning;

    public void buildPaths(){
        turning = new Path(new BezierLine(STARTPOSE, TURN));
        turning.setConstantHeadingInterpolation(STARTPOSE.getHeading());
    }

    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(STARTPOSE);
    }

    public void start(){
        opmodeTimer.resetTimer();
        pathState = 0;
    }

    public void autoPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(turning);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()){
                    pathState = 2;
                }
                break;
        }
    }

    public void loop(){
        follower.update();
        autoPathUpdate();
    }
}
