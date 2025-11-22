package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Testing.Test;

public class Spindex {
    private CRServo spindex = null;
    private static AnalogInput spindexPos = null;

    private double[] intakePos = {0, 240, 120};
    private double[] outtakePos = {60, 300, 180};
    private int index = 0;

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
    public void lockPos(){
        if (Math.abs(getPos()-intakePos[index%3]) <= 20){
            spindex.setPower(0);
        }
        else{
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
