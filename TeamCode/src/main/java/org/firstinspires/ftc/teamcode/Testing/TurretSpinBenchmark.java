package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Resources.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Testing.TurretSpinBenchmark.TurretSpinBenchmarkConfig.speed;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

@TeleOp
public class TurretSpinBenchmark extends OpMode {
    @Config
    public static class TurretSpinBenchmarkConfig{
        public static double speed = 1;
    }

    Turret turret;
    Follower follower;
    Pose currentPose;
    PrintWriter pen;
    ElapsedTime runtime;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.startTeleopDrive();
        turret = new Turret(hardwareMap, true);
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
        double target = turret.getTargetDeg(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        follower.setTeleOpDrive(0, 0, speed, true); // Remember, Y stick is reversed!
        turret.aimAtGoal(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

        if (runtime.milliseconds() >= 10000.0){
            pen.close();
            requestOpModeStop();
        }
        //Log
        pen.write(runtime.milliseconds() + ":" + Double.toString(Math.toDegrees(currentPose.getHeading())) + ":" + target + ":" + turret.getVelocity()/(turret.ticks*turret.gearRatio)*360.0 + "\n");
    }
}
