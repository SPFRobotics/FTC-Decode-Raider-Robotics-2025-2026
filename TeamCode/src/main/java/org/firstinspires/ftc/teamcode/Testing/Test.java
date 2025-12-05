package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFinder;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Kicker;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.LedLights;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    //private ColorFinder colorFinder = new ColorFinder(hardwareMap);
    //LedLights leftLED = new LedLights("leftLED", hardwareMap);
    //LedLights rightLED = new LedLights("rightLED", hardwareMap);
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private Spindex spindex = null;
    private Kicker kicker = null;
    private Intake intake = null;
    private Limelight limelight = new Limelight(hardwareMap);

    public static Button spindexCounterClockWise = new Button();
    public static Button spindexClockWise = new Button();
    public static Button spindexMode = new Button();
    public static Button intakeButton = new Button();

    public void runOpMode(){
        spindex = new Spindex(hardwareMap);
        kicker = new Kicker(hardwareMap);
        intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        dashboardTelemetry.setMsTransmissionInterval(1);
        telemetry.setMsTransmissionInterval(1);
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        limelight.start();
        while (opModeIsActive()){
            /*if (spindexCounterClockWise.press(gamepad1.b)){
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

            if (intakeButton.toggle(gamepad1.right_bumper)){
                intake.setPower(1);
            }
            else{
                intake.setPower(0);
            }

            if (gamepad1.dpad_up){
                outtake.setRPM(4000);
            }
            else if (gamepad1.dpad_down){
                outtake.setRPM(2700);
            }

            /*if (colorFinder.isGreen()){
                leftLED.setGreen();
                rightLED.setGreen();
            }
            else if (colorFinder.isPurple()){
                leftLED.setViolet();
                rightLED.setViolet();
            }
            else{
                leftLED.turnOFF();
                rightLED.turnOFF();
            }

            boolean outtakeMode = spindexMode.toggle(gamepad1.options);
            spindex.lockPos(outtakeMode);

            //Dashboard
            dashboardTelemetry.addData("Current Position", Spindex.getPos());
            dashboardTelemetry.addData("Target Position", spindex.targetPos);
            dashboardTelemetry.addData("Index", spindex.getIndex());
            dashboardTelemetry.addData("Mode", outtakeMode ? "OUTTAKE" : "INTAKE");
            dashboardTelemetry.update();

            //Driver Hub
            telemetry.addData("Current Position", Spindex.getPos());
            telemetry.addData("Target Position", spindex.targetPos);
            telemetry.addData("Index", spindex.getIndex());
            telemetry.addData("Mode", outtakeMode ? "OUTTAKE" : "INTAKE");
            telemetry.addData("Loop Time", timer.milliseconds());
            timer.reset();
            telemetry.update();*/

            telemetry.addData("Bot Pos", limelight.botpose());
            telemetry.update();
        }
    }
}
