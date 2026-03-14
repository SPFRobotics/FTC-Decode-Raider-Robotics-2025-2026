package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Assets.PedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;

public class PoseStorage {

    public static double IntakeSpeed = .3;
    public static Pose poseEnd = new Pose();
    public static boolean blueAlliance = false;
    public static boolean redAlliance = false;

    public static boolean turretValid = false;
    public static double turretStartPos = -1.0;

    public static void savePose(Pose pose) {
        poseEnd = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        //poseEnd = new Pose(33.000, 134.442, Math.toRadians(90));
    }

    public static void setTurretStartPos(double value) {
        turretValid = true;
        turretStartPos = value;
    }
}
