package org.firstinspires.ftc.teamcode.Game.Subsystems;

import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Testing.Test;

public class Spindex {
    @Config
    public static class SpindexValues{
        public static int p = 100;
        public static double speed = 1;
        public static double[] intakePos = {16, 136, 256};
        public static double[] outtakePos = {76, 197, 317};
    }
    private CRServo spindex = null;
    private static AnalogInput spindexPos = null;

    private boolean mode = false;
    public int index = 0;
    public double targetPos = 0;
    //Stores position and current index of spindex
    public Spindex(HardwareMap hardwareMap){
        spindex = hardwareMap.get(CRServo.class, "spindex");
        spindexPos = hardwareMap.get(AnalogInput.class, "spindexPos");
        spindex.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void addIndex(){
        index++;
    }

    public void subtractIndex(){
        index--;
    }
    //Locks on position based on the index
    public boolean getLockPos(){
        return mode;
    }

    public int getIndex(){
        return Math.floorMod(index, 3);
    }

    //Gets the minimum distance to target accepts a list and is overloaded to accept a single value if needed
    private double getMinDistance(double[] positions){
        double distance = 0;
        double target = positions[getIndex()];
        if (target + 180 > 360 ){
            if (getPos() > target){
                distance = target-getPos();
            }
            else if (getPos() <= (target + 180)%360){
                distance = -(target+getPos());
            }
            else if (getPos() < target && getPos() > (target + 180)%360){
                distance = target-getPos();
            }
        }
        else{
            if (getPos() > target && getPos() <= target+180){
                distance = target-getPos();
            }
            else if (getPos() > target+180){
                distance = (360-getPos())+target;
            }
            else if (getPos() < target){
                distance = target-getPos();
            }
        }
        return distance;
    }

    private double getMinDistance(double x){
        double distance = 0;
        double target = x;
        if (target + 180 > 360 ){
            if (getPos() > target){
                distance = target-getPos();
            }
            else if (getPos() <= (target + 180)%360){
                distance = -(target+getPos());
            }
            else if (getPos() < target && getPos() > (target + 180)%360){
                distance = target-getPos();
            }
        }
        else{
            if (getPos() > target && getPos() <= target+180){
                distance = target-getPos();
            }
            else if (getPos() > target+180){
                distance = (360-getPos())+target;
            }
            else if (getPos() < target){
                distance = target-getPos();
            }
        }
        return distance;
    }

    //False = intake, true = outtake
    public void lockPos(boolean mode){
        if (!mode){
            spindex.setPower(Math.min((getMinDistance(intakePos)/SpindexValues.p) * speed, 1));
        }
        else{
            spindex.setPower(Math.min((getMinDistance(outtakePos)/SpindexValues.p) * speed, 1));
        }
    }

    public void zero(){
        double minDistance = getMinDistance(0);
        spindex.setPower(minDistance/p);
    }

    public void move(double x){
        spindex.setPower(x);
    }

    public static double getPos(){
        return (int)(spindexPos.getVoltage() / 3.3 * 360);
    }
}