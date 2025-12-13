package org.firstinspires.ftc.teamcode.Testing;
import static org.firstinspires.ftc.teamcode.Testing.Test.Testing.PIDF;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    @Config
    public static class Testing{
        public static int[] PIDF = {10, 3, 0, 0};
    }
    public void runOpMode(){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDF[0], PIDF[1], PIDF[2], PIDF[3]));
            //motor.setPower();
            telemetry.addData("Velocity", motor.getVelocity());
            telemetry.addData("RPM", motor.getVelocity()*60/28);
            telemetry.addData("PID", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }
    }
}
