package org.firstinspires.ftc.teamcode.Testing;
import static org.firstinspires.ftc.teamcode.Testing.Test.Testing.PIDF;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Resources.Scroll;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    @Config
    public static class Testing{
        public static int[] PIDF = {10, 3, 0, 0};
    }
    private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");
    private Scroll daddyRyan = new Scroll("Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.");
    public void runOpMode(){
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "spindex");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(PIDF[0], PIDF[1], PIDF[2], PIDF[3]));
            motor.setPower(gamepad1.right_trigger);
            telemetry.addLine(bigThree.foward());
            telemetry.addData("Velocity", motor.getVelocity());
            telemetry.addData("RPM", motor.getVelocity()*60/537.7);
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("PID", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
            telemetry.addLine(daddyRyan.foward());
        }
    }
}
