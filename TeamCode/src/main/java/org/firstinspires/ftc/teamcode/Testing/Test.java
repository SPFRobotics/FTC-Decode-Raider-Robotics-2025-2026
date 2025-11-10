package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Game.Spindex;
@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    public ColorSensor colorSensor = null;
    public DistanceSensor distanceSensor = null;
    private CRServo servo = null;
    private AnalogInput encoder = null;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry telemetry = dashboard.getTelemetry();
    private ElapsedTime clock = new ElapsedTime();
    private Spindex spindex = null;
    public DcMotor Motor1 = null;
    public void runOpMode(){
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");
        servo = hardwareMap.get(CRServo.class, "servo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        spindex = new Spindex(hardwareMap);
        int red = 0;
        int green = 0;
        int blue = 0;

        //Motor1 = hardwareMap.get(DcMotor.class, Testing.motor);
        waitForStart();
        while (opModeIsActive()){
            red = Math.min(colorSensor.red(), 255);
            green = Math.min(colorSensor.green(), 255);
            blue = Math.min(colorSensor.blue(), 255);

            if (gamepad1.a){
                spindex.zero();
            }
            else{
                spindex.move(gamepad1.right_trigger);
            }
            telemetry.addData("Color Code ",red + ", " + green + ", " + blue);
            if (red >= 40 && red <= 50 && green >= 100 && green <= 110 && blue >= 80 && blue <= 90){
                telemetry.addData("Color ", "Green");
            }
            else if ((red >= 50 && red <= 58 && green >= 80 && green <= 90 && blue >= 90 && blue <= 100)){
                telemetry.addData("Color ", "Purple");
            }
            else{
                telemetry.addData("Color ", "Other");
            }
            telemetry.addData("Color: ", "Other");
            telemetry.addData("Distance ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoder ", spindex.getPos());
            telemetry.update();
        }
    }
}
