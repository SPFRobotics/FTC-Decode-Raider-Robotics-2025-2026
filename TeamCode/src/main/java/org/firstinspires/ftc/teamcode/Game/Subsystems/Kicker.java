package org.firstinspires.ftc.teamcode.Game.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Kicker.KickerConfig.*;


public class Kicker {
    @Config
    public static class KickerConfig{
        public static double downGrav = 0.05;
        public static double upGrav = 0.18;

        public static double down = 0.577;
        public static double up = 0.8;


    }
    @Config
    public static class KickerEncoderTargets{
        public static double downAngleDegrees = 20;
        public static double upAngleDegrees = 210;
        public static double toleranceDegrees = 8;
    }
    private Servo kicker = null;
    private Servo leftKicker = null;
    private Servo rightKicker = null;
    private static int state = 0;
    private boolean grav = false;


    //Accepts boolean for if we are using the gravity feed or spindex design
    public Kicker(HardwareMap hardwareMap, boolean grav){
        this.grav = grav;
        if (grav){
            kicker = hardwareMap.get(Servo.class, "kicker");
            kicker.setDirection(Servo.Direction.REVERSE);
        }
        else{
            leftKicker = hardwareMap.get(Servo.class, "leftKicker");
            rightKicker = hardwareMap.get(Servo.class, "rightKicker");
            leftKicker.setDirection(Servo.Direction.REVERSE);
        }
    }

    public void up(){
        state = 1;
        if (grav){
            kicker.setPosition(upGrav);
        }
        else{
            leftKicker.setPosition(up);
            rightKicker.setPosition(up);
        }
    }

    public void down(){
        state = 0;
        if (grav){
            kicker.setPosition(downGrav);
        }
        else{
            leftKicker.setPosition(down);
            rightKicker.setPosition(down);
        }
    }

    public void zero(){
        state = 0;
        if (grav){
            kicker.setPosition(0);
        }
        else{
            leftKicker.setPosition(0);
            rightKicker.setPosition(0);
        }
    }

    public static int getState(){
        return state;
    }
}
