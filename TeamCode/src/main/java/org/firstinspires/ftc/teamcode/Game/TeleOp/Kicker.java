package org.firstinspires.ftc.teamcode.Game.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Kicker {
    @Config
    public static class KickerHeight{
        public static double down = 0.05;
        public static double up = 0.18;

    }
    private Servo kicker = null;
    private boolean state = false;

    private AnalogInput voltage = null;

    public Kicker(HardwareMap hardwareMap){
        kicker = hardwareMap.get(Servo.class, "Kicker");
        voltage = hardwareMap.get(AnalogInput.class, "KickerPos");
        kicker.setDirection(Servo.Direction.REVERSE);
    }

    public void up(){
        state = true;
        kicker.setPosition(KickerHeight.up);
    }

    public void down(){
        state = false;
        kicker.setPosition(KickerHeight.down);
    }

    public void zero(){
        state = false;
        kicker.setPosition(0);

    }

    public double volts(){
         return voltage.getVoltage();

    }
    
    public boolean getState(){
        return state;
    }

}
