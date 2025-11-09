package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    public ColorSensor colorSensor = null;
    public DistanceSensor distanceSensor = null;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry telemetry = dashboard.getTelemetry();
    private ElapsedTime clock = new ElapsedTime();
    public DcMotor Motor1 = null;
    public void runOpMode(){
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");
        //Motor1 = hardwareMap.get(DcMotor.class, Testing.motor);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Color: ", colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.red());
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
