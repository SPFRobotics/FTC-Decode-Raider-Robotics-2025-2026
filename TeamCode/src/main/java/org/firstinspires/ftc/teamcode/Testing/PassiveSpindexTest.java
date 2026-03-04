package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.PassiveKicker;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import static org.firstinspires.ftc.teamcode.Testing.PassiveSpindexTest.PassiveSpindexTestConfig.*;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

@TeleOp
public class PassiveSpindexTest extends OpMode {
    Spindex spindex;
    KickerSpindex kicker;
    PassiveKicker passiveKicker;
    Outtake outtake;
    Intake intake;
    Button kickerButton = new Button();
    PrintWriter pen = null;
    ElapsedTime runTime = null;
    AnalogInput powerSwitch = null;
    double maxVoltage = 0;



    @Config
    public static class PassiveSpindexTestConfig{
        public static double RPM = Outtake.OuttakeConfig.closeRPM;
        public static double speedMultiplyer = 1;
    }

    public void init(){
        spindex = new Spindex(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        passiveKicker = new PassiveKicker(hardwareMap);
        powerSwitch = hardwareMap.get(AnalogInput.class, "kickstandPos");
        try{
            pen = new PrintWriter("/sdcard/outtake.txt", "ASCII");
        }
        catch (FileNotFoundException e){

        }
        catch (UnsupportedEncodingException e){

        }
        intake.setPower(1);
    }

    public void start(){
        outtake.setRPM(RPM);
        runTime = new ElapsedTime();
    }

    public void loop(){
        //kicker.automate(kickerButton.press(gamepad1.a));

        if (gamepad1.dpadDownWasPressed()){
            passiveKicker.down();
        }
        else if (gamepad1.dpadUpWasPressed()){
            passiveKicker.up();
        }

        kicker.passive();
        spindex.setPower((-gamepad1.right_trigger + gamepad1.left_trigger)*speedMultiplyer);
        pen.write(runTime.milliseconds() + ":" + outtake.getRPM() + "\n");

        maxVoltage = powerSwitch.getVoltage() > maxVoltage ? maxVoltage = powerSwitch.getVoltage() : maxVoltage;
        telemetry.addLine("Max Voltage: " + maxVoltage);
        telemetry.update();
    }

    public void stop(){
        pen.close();
    }
}
