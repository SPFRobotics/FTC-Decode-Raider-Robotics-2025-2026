package org.firstinspires.ftc.teamcode.Assets;
import static org.firstinspires.ftc.teamcode.Assets.PedroPathing.Tuning.follower;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PedroMethods {

    /**
     * @Author ZuckyWucky
     * @param apriltag Limelight's field coordinates
     * @returns Pedro Field Coordinates
     */
    public Pose limelightToPedroCoords(Pose2D apriltag){
        Pose ftcStandard = PoseConverter.pose2DToPose(apriltag, InvertedFTCCoordinates.INSTANCE);

        return  (ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE));

    }
}