package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import static org.firstinspires.ftc.teamcode.Testing.PassiveSpindexTest.PassiveSpindexTestConfig.*;

public class PassiveSpindexTest extends OpMode {
    Spindex spindex;
    KickerSpindex kicker;
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
    }

    public void start(){
        outtake.setRPM(RPM);
    }

    public void loop(){
        kicker.automate(kickerButton.press(gamepad1.a));

        spindex.setPower(-gamepad1.right_trigger + gamepad1.left_trigger);
    }
}
