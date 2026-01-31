package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//ROYGBIV!!!
public class LedLights {
    ElapsedTime colorTimer = new ElapsedTime();
    double currentColor = 0;
    double precision = 0.001;
    boolean disabled = false;

    Servo leftLed = null;
    Servo rightLed = null;


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

    private void setLeds(double pwm){
        leftLed.setPosition(pwm);
        rightLed.setPosition(pwm);
    }

//Constructor
    public LedLights(HardwareMap hardwareMap) {
        leftLed = hardwareMap.get(Servo.class, "leftLed");
        rightLed = hardwareMap.get(Servo.class, "rightLed");
    }

    //One interval is the amount of time in seconds it takes to get back to the value of
    public void cycleColors(int interval){
        if (colorTimer.seconds() >= interval/495.0/2){
            colorTimer.reset();
            currentColor += precision;
            if (currentColor >= VIOLET){
                currentColor = VIOLET;
                precision *= -1;
            }
            else if (currentColor <= RED){
                currentColor = RED;
                precision *= -1;
            }
            setLeds(currentColor);
        }
    }

    //A cycle is defined as returning to 0
    public void blink(double pwm, double hz){
        if (hz < 2.0){
            throw new RuntimeException("LEDs may not flash at a rate smaller than 2hz as advised by the game manual");
        }

        if (colorTimer.seconds() >= hz) {
            currentColor = pwm;
            colorTimer.reset();
        }
        else if (colorTimer.seconds() >= hz / 2.0) {
            currentColor = 0;
        }
        setLeds(currentColor);
    }


    public void setColor(double pwm){
        if (colorTimer.seconds() >= 2.0){
            colorTimer.reset();
            setLeds(pwm);
        }
    }

    public void setColor(double pwm, boolean wait){
        if (colorTimer.seconds() >= 2.0 && wait){
            colorTimer.reset();
            setLeds(pwm);
        }
        else{
            setLeds(pwm);
        }
    }

    public void disableLeds(){
        disabled = true;
    }

    public void enableLeds(){
        disabled = false;
    }

    public boolean getLedsState(){
        return disabled;
    }
}
