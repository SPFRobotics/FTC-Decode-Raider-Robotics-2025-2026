package org.firstinspires.ftc.teamcode.Game.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex.KickerConfig.*;


public class KickerSpindex {
    @Config
    public static class KickerConfig{
        public static double down = 0.577;
        public static double up = 0.8;
    }
    private Servo leftKicker = null;
    private Servo rightKicker = null;
    private static int state = 0;


    //Accepts boolean for if we are using the gravity feed or spindex design
    public KickerSpindex(HardwareMap hardwareMap){
        leftKicker = hardwareMap.get(Servo.class, "leftKicker");
        rightKicker = hardwareMap.get(Servo.class, "rightKicker");
        leftKicker.setDirection(Servo.Direction.REVERSE);
    }

    public void up(){
        state = 1;
        leftKicker.setPosition(up);
        rightKicker.setPosition(up);
    }

    public void down(){
        state = 0;
        leftKicker.setPosition(down);
        rightKicker.setPosition(down);
    }

    public void zero(){
        state = 0;
        leftKicker.setPosition(0);
        rightKicker.setPosition(0);
    }

    public static int getState(){
        return state;
    }
}
