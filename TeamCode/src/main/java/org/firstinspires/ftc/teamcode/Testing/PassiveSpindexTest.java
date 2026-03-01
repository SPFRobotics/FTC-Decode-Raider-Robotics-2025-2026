package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.PassiveKicker;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import static org.firstinspires.ftc.teamcode.Testing.PassiveSpindexTest.PassiveSpindexTestConfig.*;

@TeleOp
public class PassiveSpindexTest extends OpMode {
    Spindex spindex;
    KickerSpindex kicker;
    PassiveKicker passiveKicker;
    Outtake outtake;
    Button kickerButton = new Button();

    @Config
    public static class PassiveSpindexTestConfig{
        public static double RPM = Outtake.OuttakeConfig.closeRPM;
    }

    public void init(){
        spindex = new Spindex(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        outtake = new Outtake(hardwareMap);
        passiveKicker = new PassiveKicker(hardwareMap);
    }

    public void start(){
        outtake.setRPM(RPM);
    }

    public void loop(){
        kicker.automate(kickerButton.press(gamepad1.a));

        if (gamepad1.dpadDownWasPressed()){
            passiveKicker.down();
        }
        else if (gamepad1.dpadUpWasPressed()){
            passiveKicker.up();
        }

        spindex.setPower(-gamepad1.right_trigger + gamepad1.left_trigger);
    }
}
