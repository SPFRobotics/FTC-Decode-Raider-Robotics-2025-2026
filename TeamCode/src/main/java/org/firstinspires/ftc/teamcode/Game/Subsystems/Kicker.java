package org.firstinspires.ftc.teamcode.Game.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Kicker {
    @Config
    public static class KickerHeight{
        public static double downGrav = 0.05;
        public static double upGrav = 0.18;

        public static double down = 0.3;
        public static double up = 0.53;


    }
    private Servo kicker = null;
    private static int state = 0;

    private AnalogInput voltage = null;

    public Kicker(HardwareMap hardwareMap){
        kicker = hardwareMap.get(Servo.class, "Kicker");
        voltage = hardwareMap.get(AnalogInput.class, "KickerPos");
        kicker.setDirection(Servo.Direction.REVERSE);
    }

    public void up(boolean grav){
        state = 1;
        if (grav){
            kicker.setPosition(KickerHeight.upGrav);
        }
        else{
            kicker.setPosition(KickerHeight.up);
        }
    }

    public void down(boolean grav){
        state = 0;
        if (grav){
            kicker.setPosition(KickerHeight.downGrav);
        }
        else{
            kicker.setPosition(KickerHeight.down);
        }
    }

    public void zero(){
        state = 0;
        kicker.setPosition(0);

    }

    public double volts(){
         return voltage.getVoltage();

    }

    public static int getState(){
        return state;
    }

}
