package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    public ColorSensor colorsensor = null;
    public DcMotor Motor1 = null;
    public void runOpMode(){
        colorsensor = hardwareMap.get(ColorSensor.class, "sensor");
        //Motor1 = hardwareMap.get(DcMotor.class, Testing.motor);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Color: ", colorsensor.red() + ", " + colorsensor.green() + ", " + colorsensor.red());
            telemetry.update();
        }
    }
}
