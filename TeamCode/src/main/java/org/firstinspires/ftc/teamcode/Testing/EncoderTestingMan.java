package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
public class EncoderTestingMan extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        String backLeft = "backLeftDrive";
        String backRight = "backRightDrive";
        String frontLeft = "frontLeftDrive";
        String frontRight = "frontRightDrive";

        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, backLeft);
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, backRight);
        DcMotor frontLeftMotor  = hardwareMap.get(DcMotor.class, frontLeft);
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, frontRight);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()){
            telemetry.addLine("This program is used to test if the encoders of each motor are plugged into the correct port.\n");
            telemetry.addLine("To use, press either the cross, circle, triangle, or square buttons and read the encoder output\n");
            telemetry.addLine("If the encoder does not correlate with the motor, pressing the button corresponding to motor will result in the encoder count staying static");
            telemetry.addLine("==========================================");
            if (gamepad1.cross){
                backLeftMotor.setPower(1);
                telemetry.addData(backLeft + " Pos", backLeftMotor.getCurrentPosition());
            }
            else if (gamepad1.circle){
                backRightMotor.setPower(1);
                telemetry.addData(backRight + " Pos", backRightMotor.getCurrentPosition());
            }
            else if (gamepad1.triangle){
                frontLeftMotor.setPower(1);
                telemetry.addData(frontLeft + " Pos", frontLeftMotor.getCurrentPosition());
            }
            else if (gamepad1.square){
                frontRightMotor.setPower(1);
                telemetry.addData(frontRight + " Pos", frontRightMotor.getCurrentPosition());
            }
            else{
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
            }
            telemetry.update();
        }
    }
}
