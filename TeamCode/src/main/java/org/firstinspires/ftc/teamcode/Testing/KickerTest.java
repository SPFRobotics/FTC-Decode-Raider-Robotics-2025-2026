package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;

public class KickerTest extends OpMode {

    KickerSpindex kicker;

    @Override
    public void init() {
        kicker = new KickerSpindex(hardwareMap);
    }

    @Override
    public void start() {

    }


    public void loop(){

        if(gamepad1.dpad_up){
            kicker.up();
        }

        if(gamepad1.dpad_down){
            kicker.down();
        }
}
}
