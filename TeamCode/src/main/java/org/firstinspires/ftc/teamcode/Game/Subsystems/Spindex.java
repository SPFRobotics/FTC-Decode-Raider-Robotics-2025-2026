package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Testing.Test;

public class Spindex {
    @Config
    public static class SpindexValues{
        public static int range = 0;
    }
    private CRServo spindex = null;
    private static AnalogInput spindexPos = null;

    private double[] intakePos = {0, 240, 120};
    private double[] outtakePos = {60, 300, 180};
    private int index = 0;
    private double distance = 0;
    //Stores position and current index of spindex
    public Spindex(HardwareMap hardwareMap){
        spindex = hardwareMap.get(CRServo.class, "servo");
        spindexPos = hardwareMap.get(AnalogInput.class, "encoder");
    }

    public void addIndex(){
        index++;
    }

    public void subtractIndex(){
        index--;
    }
    //Locks on position based on the index

    public void lockPos(boolean mode){
        if (!mode){
            distance = getPos()-intakePos[index%3];
        }
        else{
            distance = getPos()-outtakePos[index%3];
        }

        if (Math.abs(distance) < SpindexValues.range){
            spindex.setPower(0);
        }
        else {
            spindex.setPower(0.1);
        }
    }

    public void zero(){
        if (Spindex.getPos() <= 20){
            spindex.setPower(0);
        }
        else{
            spindex.setPower(0.1);
        }
    }

    public void move(double x){
        spindex.setPower(x);
    }

    public static double getPos(){
        return (int)(spindexPos.getVoltage() / 3.3 * 360);
    }
}
