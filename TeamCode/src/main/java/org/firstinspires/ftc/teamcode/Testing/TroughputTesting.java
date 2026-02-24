package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Testing.TroughputTesting.Config.power;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
        //(name="Throughput Testing")

public class TroughputTesting extends LinearOpMode {
    public static class Config{
        public static double power = 1;
    }

    public void runOpMode(){
        DcMotor motor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.right_trigger > 0) {
                motor.setPower(power);
            }
            else{
                motor.setPower(0);
            }
        }
    }
}
