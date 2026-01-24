package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.LedLights;

@TeleOp(name="SLOTHING IT")
public class LedStrobe extends OpMode{
    Servo leftLed = null;
    Servo rightLed = null;
    LedLights rightLedCtrl = null;
    LedLights leftLedCtrl = null;

    public void init(){
        leftLed = hardwareMap.get(Servo.class, "leftLed");
        rightLed = hardwareMap.get(Servo.class, "rightLed");

        leftLedCtrl = new LedLights(leftLed);
        rightLedCtrl = new LedLights(rightLed);
    }

    public void loop(){
       leftLedCtrl.cycleColors(10);
       rightLedCtrl.cycleColors(10);
    }
}
