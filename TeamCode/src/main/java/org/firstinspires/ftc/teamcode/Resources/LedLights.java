package org.firstinspires.ftc.teamcode.Resources;

import static java.lang.System.exit;
import static java.lang.System.in;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

//ROYGBIV!!!
public class LedLights {
    Servo led = null;
    ElapsedTime colorTimer = new ElapsedTime();
    double currentColor = 0;
    double precision = 0.001;

    /*Constants*/
    public final double RED = 0.279;
    public final double ORANGE = 0.333;
    public final double YELLOW = 0.388;
    public final double SAGE = 0.444;
    public final double GREEN = 0.500;
    public final double AZURE = 0.555;
    public final double BLUE = 0.611;
    public final double INDIGO = 0.666;
    public final double VIOLET = 0.722;


//Constructor
    public LedLights(Servo led) {
        this.led = led;
    }

    //One interval is the amount of time in seconds it takes to get back to the value of
    public void cycleColors(int interval){
        if (colorTimer.seconds() >= interval/495.0/2){
            colorTimer.reset();
            currentColor += precision;
            if (currentColor >= 0.722){
                currentColor = 0.722;
                precision *= -1;
            }
            else if (currentColor <= 0.279){
                currentColor = 0.279;
                precision *= -1;
            }
            led.setPosition(currentColor);
        }
    }

    public void setColor(double pwd){
        led.setPosition(pwd);
    }
}
