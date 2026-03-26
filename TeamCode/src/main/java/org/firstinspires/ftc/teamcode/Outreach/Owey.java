package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Plowie")
public class Owey extends LinearOpMode {

    private DcMotor LeftWheel;
    private DcMotor RightWheel;
    private Servo RightArm;
    private Servo LeftArm;

    @Override
    public void runOpMode() {
        int Servo_Angle;

        RightWheel = hardwareMap.get(DcMotor.class, "Left Wheel");
        LeftWheel = hardwareMap.get(DcMotor.class, "RightWheel");
        RightArm = hardwareMap.get(Servo.class, "RightArm");
        LeftArm = hardwareMap.get(Servo.class, "LeftArm");

        waitForStart();

        LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightArm.setDirection(Servo.Direction.REVERSE);

        RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);



        Servo_Angle = 40;

        while (opModeIsActive()) {

            // ----- Servo Arm Control -----
            if (gamepad1.dpad_up) Servo_Angle++;
            if (gamepad1.dpad_down) Servo_Angle--;

            Servo_Angle = Math.max(40, Math.min(90, Servo_Angle));
            double servoPos = Servo_Angle / 360.0 + 0.2;

            LeftArm.setPosition(servoPos);
            RightArm.setPosition(servoPos);

            double speedMultipler = gamepad1.right_bumper && gamepad1.left_bumper ? 1 : 0.5;


            LeftWheel.setPower(gamepad1.left_stick_y * speedMultipler);
            RightWheel.setPower(gamepad1.right_stick_y * speedMultipler);

            telemetry.addData("Left Power:", LeftWheel.getPower());
            telemetry.addData("Right Power:", LeftWheel.getPower());
            telemetry.update();
        }
    }
}
