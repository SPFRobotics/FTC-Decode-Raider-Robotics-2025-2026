package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.Button;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    //public ColorSensor colorSensor = null;
    //public Servo led = null;
    //public DistanceSensor distanceSensor = null;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry telemetry = dashboard.getTelemetry();
    private Spindex spindex = null;
    public static Button spindexCounterClockWise = new Button();
    public static Button spindexClockWise = new Button();
    public static Button spindexMode = new Button();

    @Config
    public static class IndexerConfig{
        public static double speed = 0.1;
        public static int range = 5;
    }

    public void runOpMode(){
        spindex = new Spindex(hardwareMap);
        telemetry.setMsTransmissionInterval(1);
        waitForStart();
        while (opModeIsActive()){
            Spindex.SpindexValues.range = IndexerConfig.range;
            Spindex.SpindexValues.speed = IndexerConfig.speed;

            if (spindexCounterClockWise.press(gamepad1.b)){
                spindex.addIndex();
            }
            else if (spindexClockWise.press(gamepad1.x)){
                spindex.subtractIndex();
            }

            boolean outtakeMode = spindexMode.toggle(gamepad1.options);
            spindex.lockPos(outtakeMode);

            telemetry.addData("Current Position", Spindex.getPos());
            telemetry.addData("Mode", outtakeMode ? "OUTTAKE" : "INTAKE");
            telemetry.addData("Dashboard Speed", IndexerConfig.speed);
            telemetry.addData("Dashboard Range", IndexerConfig.range);
            telemetry.update();
        }
    }
}
