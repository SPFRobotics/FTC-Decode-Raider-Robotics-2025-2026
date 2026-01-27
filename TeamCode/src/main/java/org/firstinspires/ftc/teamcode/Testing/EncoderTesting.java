package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class EncoderTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeftDrive");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRightDrive");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRightDrive");

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setTargetPosition(537);
        backRight.setTargetPosition(537);
        frontLeft.setTargetPosition(537);
        frontRight.setTargetPosition(537);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        backLeft.setPower(0.1);
        backRight.setPower(0.1);
        frontLeft.setPower(0.1);
        frontRight.setPower(0.1);

        while (opModeIsActive() && backRight.isBusy() && backLeft.isBusy() && frontRight.isBusy() && frontLeft.isBusy()){
        }

        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
}
