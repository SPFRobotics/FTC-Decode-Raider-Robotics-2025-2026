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

        public static double down = 0.575;
        public static double up = 1;


    }
    @Config
    public static class KickerEncoderTargets{
        public static double downAngleDegrees = 20;
        public static double upAngleDegrees = 210;
        public static double toleranceDegrees = 8;
    }
    private Servo kicker = null;
    private static int state = 0;

    private static AnalogInput voltage = null;

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

    public static int getEncoderState(){
        if (voltage == null){
            return state;
        }
        double angle = getPos();
        if (isWithinTolerance(angle, KickerEncoderTargets.upAngleDegrees)){
            return 1;
        }
        if (isWithinTolerance(angle, KickerEncoderTargets.downAngleDegrees)){
            return 0;
        }
        return state;
    }

    public static double getPos(){
        return (int)(voltage.getVoltage() / 3.3 * 360);
    }

    private static boolean isWithinTolerance(double current, double target){
        double diff = Math.abs(current - target);
        if (diff > 180){
            diff = 360 - diff;
        }
        return diff <= KickerEncoderTargets.toleranceDegrees;
    }

}
