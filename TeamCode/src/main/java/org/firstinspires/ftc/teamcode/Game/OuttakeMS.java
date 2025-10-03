package org.firstinspires.ftc.teamcode.Game;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class OuttakeMS extends LinearOpMode {

    private Servo outtakeServo;

    private DcMotor outtakeMotor;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOuttake(double power) {
        outtakeMotor.setPower(power);
    }

    public void stopOuttake() {
        outtakeMotor.setPower(0);
    }

    public void Servo(){
        outtakeServo.setPosition(0.028);
    }

    public void runOpMode() {

        outtakeMotor = hardwareMap.get(DcMotor.class, "outtake_motor");
        waitForStart();
        while (opModeIsActive()) {

            Servo();
            double outtakePower = gamepad1.left_stick_y;
            runOuttake(outtakePower);

            if (gamepad1.a) {
                stopOuttake();


            }
            telemetry.addLine("----Outake----");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Outtake Power", outtakePower);
            telemetry.update();

        }
    }
}
