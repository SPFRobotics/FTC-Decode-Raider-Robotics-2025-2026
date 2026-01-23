package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.LedLights;

@TeleOp(name="SLOTHING IT")
public class LedStrobe extends OpMode{
    Servo led = null;
    double color = 0.279;
    ElapsedTime timer = new ElapsedTime();
    //One cycle occurs when the LED returns to 0.279
    double ledCycle = 10;
    double precision = 0.001;

    public void init(){
        //LedLights leds = new LedLights(hardwareMap, led);
        led = hardwareMap.get(Servo.class, "led");
    }

    public void start(){
        timer.reset();
    }

    public void loop(){
        if (timer.seconds() >= ledCycle/495.0/2){
            timer.reset();
            color += precision;
            if (color >= 0.722){
                color = 0.722;
                precision *= -1;
            }
            else if (color <= 0.279){
                color = 0.279;
                precision *= -1;
            }
            led.setPosition(color);
        }
    }
}
