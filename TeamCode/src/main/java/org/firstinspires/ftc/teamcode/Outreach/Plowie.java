package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Plowie extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;

    Servo leftArm;
    Servo rightArm;

    double xPower;
    double yPower;


    @Override
    public void init() {
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftWheel = hardwareMap.get(DcMotor.class, "leftWheel");
        rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");
    }

    @Override
    public void loop() {

        xPower= gamepad1.left_stick_x;
        yPower= gamepad1.left_stick_y;

        leftWheel.setPower(xPower+yPower);
        rightWheel.setPower(xPower-yPower);

        if(gamepad1.dpadUpWasPressed()){
            leftArm.setPosition(45);
            rightArm.setPosition(45);
        }
        if(gamepad1.dpadDownWasPressed()){
            leftArm.setPosition(0);
            rightArm.setPosition(0);
        }


    }
}
