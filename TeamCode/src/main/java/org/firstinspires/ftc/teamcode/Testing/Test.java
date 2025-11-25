package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Kicker;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.Button;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    //public ColorSensor colorSensor = null;
    //public Servo led = null;
    //public DistanceSensor distanceSensor = null;
    private CRServo servo = null;
    private AnalogInput encoder = null;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry telemetry = dashboard.getTelemetry();
    private ElapsedTime clock = new ElapsedTime();
    private Spindex spindex = null;
    public static Button spindexCounterClockWise = new Button();
    public static Button spindexClockWise = new Button();
    public static Button spindexMode = new Button();
    public DcMotor Motor1 = null;
    public ElapsedTime ledClock = new ElapsedTime();

    @Config
    public static class Testing{
        public static double led1 = 0.5;
        public static double led2 = 0.67;
    }

    public void runOpMode(){
        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");
        servo = hardwareMap.get(CRServo.class, "servo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        spindex = new Spindex(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Kicker kicker = new Kicker(hardwareMap);
        telemetry.setMsTransmissionInterval(1);
        int r = 0,  g = 0, b = 0;
        int[] HSV = new int[3];

        //Motor1 = hardwareMap.get(DcMotor.class, Testing.motor);
        waitForStart();
        while (opModeIsActive()){
            //Math.min() is used to cap the max RGB value at 255. Any value that is higher wouldn't make any sense with the RGB model
            //r = Math.min(colorSensor.red(), 255);
            //g = Math.min(colorSensor.green(), 255);
            //b = Math.min(colorSensor.blue(), 255);


            if (spindexCounterClockWise.press(gamepad1.b)){
                spindex.addIndex();
            }
            else if (spindexClockWise.press(gamepad1.x)){
                spindex.subtractIndex();
            }

            if (spindexMode.toggle(gamepad1.options)){
                spindex.lockPos(true);
            }
            else{
                spindex.lockPos(false);
            }

            if (gamepad1.right_trigger > 0){
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }

            if (gamepad1.a){
                kicker.down();
            }
            else if (gamepad1.y){
                kicker.up();
            }

            telemetry.addData("RGB ",r + ", " + g + ", " + b);
            telemetry.addData("HSV ",HSV[0] + ", " + HSV[1] + "%, " + HSV[2] + "%");
            /*if (HSV[0] >= 155 && HSV[0] <= 160){
                telemetry.addData("Color ", "Green");
                led.setPosition(Testing.led1);
            }
            else if ((HSV[0] >= 200 && HSV[0] <= 220)){
                telemetry.addData("Color ", "Purple");
                led.setPosition(Testing.led2);
            }
            else{
                telemetry.addData("Color ", "Other");
                led.setPosition(1);
            }*/
            //telemetry.addData("Color: ", "Other");
            //telemetry.addData("Distance ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoder ", spindex.getPos());
            telemetry.addData("Current Position", Spindex.getPos());
            telemetry.addData("Target Position", spindex.getTargetPosition());
            telemetry.addData("Error (deg)", spindex.getDistanceToTarget());
            telemetry.addData("Servo Power", spindex.getLastCommandedPower());
            //telemetry.addData("Distance ", spindex.getDistanceToSetPos());
            telemetry.update();
        }
    }
}
