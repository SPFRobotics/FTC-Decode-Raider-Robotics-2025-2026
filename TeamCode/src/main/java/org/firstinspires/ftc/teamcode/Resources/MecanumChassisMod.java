package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MecanumChassisMod {
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private IMU imu = null;

    public MecanumChassisMod(HardwareMap hardwareMap) {
        backLeft = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRight = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightDrive");
        imu = hardwareMap.get(IMU.class, "imu");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public double inch_convert(double inch) {
        return inch * (537.7 / (3.78 * Math.PI));
    }

    public void setTargetAll(double moveDistance){
        backLeft.setTargetPosition((int) inch_convert(moveDistance));
        backRight.setTargetPosition((int) inch_convert(moveDistance));
        frontLeft.setTargetPosition((int) inch_convert(moveDistance));
        frontRight.setTargetPosition((int) inch_convert(moveDistance));
    }
}
