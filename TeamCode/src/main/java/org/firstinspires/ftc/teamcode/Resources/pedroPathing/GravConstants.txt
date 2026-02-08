package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(10.25)
            .forwardZeroPowerAcceleration(-34.83144111804443)
            .lateralZeroPowerAcceleration(53.31811698147531)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0.022,0))
            ;



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8)
            .rightFrontMotorName("frontRightDrive")
            .rightRearMotorName("backRightDrive")
            .leftRearMotorName("backLeftDrive")
            .leftFrontMotorName("frontLeftDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(55.59317897615013)
            .yVelocity(35.39638307253173)
            ;


    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.0019882943822326186)
            .strafeTicksToInches(0.0019882943822326186)
            .turnTicksToInches(0.0679857024584986)
            .leftPodY(2.5)
            .rightPodY(-2.5)
            .strafePodX(-3.5)
            .leftEncoder_HardwareMapName("backLeftDrive")
            .rightEncoder_HardwareMapName("frontRightDrive")
            .strafeEncoder_HardwareMapName("backRightDrive")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .threeWheelIMULocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
