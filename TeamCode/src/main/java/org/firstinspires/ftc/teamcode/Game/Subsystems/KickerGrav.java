package org.firstinspires.ftc.teamcode.Game.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.KickerGrav.KickerConfig.*;


public class KickerGrav {
    @Config
    public static class KickerConfig{
        public static double down = 0.05;
        public static double up = 0.18;
    }
    private Servo kicker = null;
    private static int state = 0;


    //Accepts boolean for if we are using the gravity feed or spindex design
    public KickerGrav(HardwareMap hardwareMap){
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setDirection(Servo.Direction.REVERSE);
    }

    public void up(){
        state = 1;
        kicker.setPosition(up);
    }

    public void down(){
        state = 0;
        kicker.setPosition(down);
    }

    public void zero(){
        state = 0;
        kicker.setPosition(0);
    }

    public static int getState(){
        return state;
    }
}
