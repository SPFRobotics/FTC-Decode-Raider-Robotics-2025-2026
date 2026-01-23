package org.firstinspires.ftc.teamcode.Resources;

import static java.lang.System.exit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

//ROYGBIV!!!
public class LedLights {
    private static class NullObjectReference extends RuntimeException{
        public NullObjectReference(){
            super("Null Object Reference");
        }
    }
    ArrayList<Servo> leds = new ArrayList<Servo>();
//Constructor
    public LedLights(HardwareMap hardwareMap, Servo... x) {
        for (Servo currentServo : x){
            if (currentServo == null){
                throw new NullObjectReference();
            }
            leds.add(currentServo);
        }
    }
}
