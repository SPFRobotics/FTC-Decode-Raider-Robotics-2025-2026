package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Assets.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextTurret;

import static org.firstinspires.ftc.teamcode.Testing.TurretSpinBenchmark.TurretSpinBenchmarkConfig.speed;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
@Disabled
@TeleOp
public class TurretSpinBenchmark extends OpMode {
    @Config
    public static class TurretSpinBenchmarkConfig{
        public static double speed = 1;
    }

    NextTurret turret = NextTurret.INSTANCE;
    Follower follower;
    Pose currentPose;
    PrintWriter pen;
    ElapsedTime runtime;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.startTeleopDrive();
        turret.setGoalCoords(true);
        turret.initialize();
        try {
            pen = new PrintWriter("/sdcard/turret.txt", "ASCII");
        }
        catch (FileNotFoundException e){
            throw new RuntimeException(e);
        }
        catch (UnsupportedEncodingException e){
            throw new RuntimeException(e);
        }
    }

    public void init_loop(){
        follower.update();
    }

    public void start(){
        runtime = new ElapsedTime();
    }

    public void loop(){
        follower.update();
        currentPose = follower.getPose();
        follower.setTeleOpDrive(0, 0, speed, true); // Remember, Y stick is reversed!
        turret.aimAtGoal(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        turret.periodic();
        double target = turret.getTargetDeg();

        if (runtime.milliseconds() >= 10000.0){
            pen.close();
            requestOpModeStop();
        }
        //Log
        pen.write(runtime.milliseconds() + ":" + Double.toString(Math.toDegrees(currentPose.getHeading())) + ":" + target + ":" + turret.getVelocity()/(NextTurret.TICKS*NextTurret.GEAR_RATIO)*360.0 + "\n");
    }
}
