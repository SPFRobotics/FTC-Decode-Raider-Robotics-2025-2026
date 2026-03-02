package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static Pose poseEnd = new Pose();
    public static boolean blueAlliance = false;
    public static boolean redAlliance = false;

    public static boolean turretValid = false;
    public static double turretStartPos = -1.0;

    public static void savePose(Pose pose) {
        poseEnd = new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static void setTurretStartPos(double value) {
        turretValid = true;
        turretStartPos = value;
    }
}
