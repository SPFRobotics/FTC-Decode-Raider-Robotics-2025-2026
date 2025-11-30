package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Kicker;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
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
    private Kicker kicker = null;
    private Intake intake = null;
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
        kicker = new Kicker(hardwareMap);
        intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        telemetry.setMsTransmissionInterval(1);
        waitForStart();
        while (opModeIsActive()){
            if (spindexCounterClockWise.press(gamepad1.b)){
                spindex.addIndex();
            }
            else if (spindexClockWise.press(gamepad1.x)){
                spindex.subtractIndex();
            }

            if (gamepad1.y){
                kicker.up(false);
            }
            else if (gamepad1.a){
                kicker.down(false);
            }

            if (gamepad1.right_bumper){
                intake.update();
            }

            if (gamepad1.left_bumper){
                intake.update();
            }

            if (gamepad1.dpad_up){
                outtake.setRPM(3200);
            }
            if (gamepad1.dpad_down){
                outtake.setRPM(0);
            }


            boolean outtakeMode = spindexMode.toggle(gamepad1.options);
            spindex.lockPos(outtakeMode);

            telemetry.addData("Current Position", Spindex.getPos());
            telemetry.addData("Target Position", spindex.targetPos);
            telemetry.addData("Index", spindex.getIndex());
            telemetry.addData("Mode", outtakeMode ? "OUTTAKE" : "INTAKE");
            telemetry.update();
        }
    }
}
