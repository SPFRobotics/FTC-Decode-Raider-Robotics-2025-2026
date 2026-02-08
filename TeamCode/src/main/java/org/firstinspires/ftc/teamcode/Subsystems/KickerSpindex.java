package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex.KickerConfig.*;


public class KickerSpindex {
    @Config
    public static class KickerConfig{
        public static double down = 0.15;
        public static double up = 0.35;
        public static double offset = 0.4;
        public static double kickerTimer = 50;
    }
    private Servo leftKicker = null;
    private Servo rightKicker = null;
    private boolean state = false;
    //True if the automate method has completed its run
    private boolean done = true;
    private ElapsedTime kickerClock = new ElapsedTime();


    //Accepts boolean for if we are using the gravity feed or spindex design
    public KickerSpindex(HardwareMap hardwareMap){
        leftKicker = hardwareMap.get(Servo.class, "kickerLeft");
        rightKicker = hardwareMap.get(Servo.class, "kickerRight");
        leftKicker.setDirection(Servo.Direction.REVERSE);
    }

    public void up(){
        state = true;
        leftKicker.setPosition(up);
        rightKicker.setPosition(up+offset);
    }

    public void down(){
        state = false;
        leftKicker.setPosition(down);
        rightKicker.setPosition(down+offset);
    }

    //Automates the motions of moving the kicker up and down combining the two methods allowing both to be done with the click of a button
    public boolean automate(boolean button){
        if (done && button){
            up();
            done = false;
            kickerClock.reset();
        }
        if (kickerClock.milliseconds() >= kickerTimer){
            down();
            done = true;
        }
        return done;
    }

    public void zero(){
        state = false;
        leftKicker.setPosition(0);
        rightKicker.setPosition(0);
    }

    public boolean getState(){
        return state;
    }
}
