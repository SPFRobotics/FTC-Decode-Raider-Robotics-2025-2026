package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Chassis {
    DcMotor frontLeftDrive;
    DcMotor backLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backRightDrive;
    public  Chassis(HardwareMap hardwareMap) {
         frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
         backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
         frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
         backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Allows speed to be halved

    public void setTeleOpDrive(double y, double x, double rx) {
            frontLeftDrive.setPower(y + x + rx);
            backLeftDrive.setPower(y - x + rx);
            frontRightDrive.setPower(y - x - rx);
            backRightDrive.setPower(y + x - rx);
    }

    public void setBrakeMode(){
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}


