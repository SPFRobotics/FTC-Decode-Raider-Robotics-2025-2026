package org.firstinspires.ftc.teamcode.Game;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



public class Kicker {
    @Config
    public static class Values{
        public static double down = 0;
        public static double up = 0.2;
    }
    private Servo kicker = null;
    private boolean state = false;

    public Kicker(HardwareMap hardwareMap){
        kicker = hardwareMap.get(Servo.class, "Kicker");
        kicker.setDirection(Servo.Direction.REVERSE);
    }

    public void up(){
        state = true;
        kicker.setPosition(Values.up);
    }

    public void down(){
        state = false;
        kicker.setPosition(Values.down);
    }

    public void zero(){
        state = false;
        kicker.setPosition(0);
    }

    public boolean getState(){
        return state;
    }

}
