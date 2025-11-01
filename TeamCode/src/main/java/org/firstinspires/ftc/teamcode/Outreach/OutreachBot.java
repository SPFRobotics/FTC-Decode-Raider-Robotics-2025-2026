package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
//@TeleOp(name="OutreachBot")
public class OutreachBot extends LinearOpMode{
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;

    public void runOpMode(){
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean cosmeticMode = false;
        boolean triggersDown = false;

        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.right_trigger == 1 && gamepad1.left_trigger == 1 && !cosmeticMode && !triggersDown){
                cosmeticMode = true;
                triggersDown = true;
            }

            if (gamepad1.right_trigger == 1 && gamepad1.left_trigger == 1 && cosmeticMode && !triggersDown){
                cosmeticMode = false;
                triggersDown = true;
            }

            if (gamepad1.right_trigger < 1 && gamepad1.left_trigger < 1){
                triggersDown = false;
            }

            if (cosmeticMode){
                rightFront.setPower(0.5);
                leftFront.setPower(-0.5);
                rightBack.setPower(0.5);
                leftBack.setPower(-0.5);

            }else{
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                rightFront.setPower((y - x - rx) / denominator);
                leftFront.setPower((y + x + rx) / denominator);
                rightBack.setPower((y + x - rx) / denominator);
                leftBack.setPower((y - x + rx) / denominator);
            }
        }
    }
}