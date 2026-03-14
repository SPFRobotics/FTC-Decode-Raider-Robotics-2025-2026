package org.firstinspires.ftc.teamcode.Assets.PedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(13.19)
            .forwardZeroPowerAcceleration(-32.8164343787902)
            .lateralZeroPowerAcceleration(-65.98248001097105)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25,0,0.02,0.027))
           .headingPIDFCoefficients(new PIDFCoefficients(3,0,0.125,0.023))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.2, 0,0.005,0.6,0.02))
            .centripetalScaling(0.004)
            ;

;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightDrive")
            .rightRearMotorName("backRightDrive")
            .leftRearMotorName("backLeftDrive")
            .leftFrontMotorName("frontLeftDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(64.23908528005043)
            .useBrakeModeInTeleOp(true)
            .yVelocity(50.18876203401821);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.875)
            .strafePodX(-7.375)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)



            ;
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
