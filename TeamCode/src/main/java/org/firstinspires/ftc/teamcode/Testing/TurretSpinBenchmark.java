package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Resources.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Testing.TurretSpinBenchmark.TurretSpinBenchmarkConfig.speed;

@TeleOp
public class TurretSpinBenchmark extends OpMode {
    @Config
    public static class TurretSpinBenchmarkConfig{
        public static double speed = 1;
    }

    Turret turret;
    Follower follower;
    Pose currentPose;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.startTeleopDrive();
        turret = new Turret(hardwareMap, true);
    }

    public void init_loop(){
        follower.update();
    }

    public void loop(){
        follower.update();
        currentPose = follower.getPose();
        follower.setTeleOpDrive(0, 0, speed, true); // Remember, Y stick is reversed!
        turret.aimAtGoal(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

        telemetry.addLine("Speed: " + speed);

    }
}
