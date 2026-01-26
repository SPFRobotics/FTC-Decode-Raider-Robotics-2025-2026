package org.firstinspires.ftc.teamcode.Game.Auto.pedroPaths;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Configurable
@Autonomous

public class BlueCloseAuto extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(34.3, 135, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(73, 76, Math.toRadians(142)); // Scoring Pose of our robot.

    private final Pose pickup1StartPose = new Pose(45, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1EndPose = new Pose(16, 85, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    //*******************************************************************
    // Launcher state machine variables
    enum LaunchState{IDLE,LOAD,LAUNCH};
    int launchBallCount = 0;
    LaunchState launchState = LaunchState.IDLE;


    //********************************************************************
    enum IntakeState{IDLE,ADVANCE,LOAD}
    int loadBallCount = 0;
    IntakeState intakeState = IntakeState.IDLE;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized");


    }


     /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        opmodeTimer.resetTimer();
        setPathState(0);
        autonomousLaunch();
        autonomousIntake();

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private Path scorePreload,startPickup1,endPickup1;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        startPickup1 = new Path(new BezierLine(scorePose, pickup1StartPose));
        startPickup1.setLinearHeadingInterpolation(scorePose.getHeading(),pickup1StartPose.getHeading());

        endPickup1 = new Path(new BezierLine(pickup1StartPose,pickup1EndPose));
        endPickup1.setLinearHeadingInterpolation(pickup1StartPose.getHeading(),pickup1EndPose.getHeading());

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // move to scoring position for preload artifacts
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1: //Case for checking to see when robot arrives at scoring position.

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    launchBallCount=3;
                    launchState= LaunchState.LOAD;
                    follower.followPath(startPickup1);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Startup intake*/
                    loadBallCount = 3;
                    intakeState=IntakeState.ADVANCE;

                    //NEED TO FIGURE OUT HOW TO CHANGE SPEED FOR THIS PATH
                    follower.followPath(endPickup1);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    setPathState(-1);
                }
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousLaunch(){
        switch(launchState){
            case IDLE:
                break;
            case LOAD:
                break;
            case LAUNCH:
                break;
        }
    }

    public void autonomousIntake(){
        switch(intakeState){
            case IDLE:
                break;
            case ADVANCE:
                break;
            case LOAD:
                break;
        }
    }
}