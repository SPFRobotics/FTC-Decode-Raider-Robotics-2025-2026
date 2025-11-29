package org.firstinspires.ftc.teamcode.Game.Subsystems;

import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.p;

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
        public static double speed = 0.1;
    }
    private CRServo spindex = null;
    private static AnalogInput spindexPos = null;

    private double[] intakePos = {0, 120, 240};
    private double[] outtakePos = {60, 300, 180};

    private boolean mode = false;
    public int index = 0;
    public double targetPos = 0;
    //Stores position and current index of spindex
    public Spindex(HardwareMap hardwareMap){
        spindex = hardwareMap.get(CRServo.class, "spindex");
        spindexPos = hardwareMap.get(AnalogInput.class, "encoder");
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

    private double getMinDistance(double[] positions){
        double distance = 0;
        if (positions[getIndex()] + 180 > 360 ){
            if (getPos() > positions[getIndex()]){
                distance = positions[getIndex()]-getPos();
            }
            else if (getPos() <= (positions[getIndex()] + 180)%360){
                distance = -(positions[getIndex()]+getPos());
            }
            else if (getPos() < positions[getIndex()] && getPos() > (positions[getIndex()] + 180)%360){
                distance = positions[getIndex()]-getPos();
            }
        }
        else{
            if (getPos() > positions[getIndex()] && getPos() <= positions[getIndex()]+180){
                distance = positions[getIndex()]-getPos();
            }
            else if (getPos() > positions[getIndex()]+180){
                distance = (360-getPos())+positions[getIndex()];
            }
            else if (getPos() < positions[getIndex()]){
                distance = positions[getIndex()]-getPos();
            }
        }
        return distance;
    }

    public void lockPos(boolean mode){
        spindex.setPower(getMinDistance(intakePos)/SpindexValues.p);
    }

    public void zero(){
        double minDistance = getMinDistance(intakePos);
        if (minDistance != 0){
            spindex.setPower(minDistance/p);
        }
    }

    public void move(double x){
        spindex.setPower(x);
    }

    public static double getPos(){
        return (int)(spindexPos.getVoltage() / 3.3 * 360);
    }
}