package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spindex {
    private CRServo spindex = null;
    private static AnalogInput spindexPos = null;
    public Spindex(HardwareMap hardwareMap){
        spindex = hardwareMap.get(CRServo.class, "servo");
        spindexPos = hardwareMap.get(AnalogInput.class, "encoder");
    }
    public void turn(){
    }

    public void zero(){
        if (Spindex.getPos() >= 0 && Spindex.getPos() <= 20){
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
