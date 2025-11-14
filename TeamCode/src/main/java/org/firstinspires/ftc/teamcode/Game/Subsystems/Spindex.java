package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spindex {
    private CRServo spindex = null;
    private static AnalogInput spindexPos = null;

    //Stores position and current index of spindex
    private double[] pos = {0, 0, 0};
    private int index = 0;
    public Spindex(HardwareMap hardwareMap){
        spindex = hardwareMap.get(CRServo.class, "servo");
        spindexPos = hardwareMap.get(AnalogInput.class, "encoder");
    }
    public void turn(){

    }

    public void zero(){
        if (0-Math.abs(Spindex.getPos()) <= 20){
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
