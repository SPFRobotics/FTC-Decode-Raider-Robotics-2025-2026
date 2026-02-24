package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;
    double f,p,i,d;
    MotorConfigurationType motorType;
    public static int stop = 0;

    boolean test = true;
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        motor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        motor.setPower(1);
        while (opModeIsActive() && test) {
            currentVelocity = motor.getVelocity();
            motor.getMotorType();
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
            if(gamepad1.left_bumper || stop==1 ){
                test = false;
                motor.setPower(0);
            }
        }
        while(opModeIsActive()){
            f =  32767/maxVelocity;
            p = 0.1*f;
            i = 0.1*p;
            d = 0.0;

            telemetry.addData("PIDF","%.2f, %.2f, %.2f, %.2f", p,i,d,f);
            telemetry.update();
        }
    }
}