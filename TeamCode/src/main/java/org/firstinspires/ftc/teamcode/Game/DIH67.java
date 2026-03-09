package org.firstinspires.ftc.teamcode.Game;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
public class DIH67 extends OpMode {

    CRServo josh;
    public void init(){
        josh = hardwareMap.get(CRServo.class, "josh");
    }
    public void  loop(){


        if(gamepad1.dpad_up) {
            josh.setPower(1);
        }else if(gamepad1.dpad_down){
            josh.setPower(-1);

        }else {
            josh.setPower(0);
        }

    }

}
