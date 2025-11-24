package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Testing.Test;

public class Spindex {
    @Config
    public static class SpindexValues{
        public static double range = 5.0; // Tolerance in degrees for position locking
    }
    private CRServo spindex = null;
    private static AnalogInput spindexPos = null;

    private double[] intakePos = {0, 240, 120};
    private double[] outtakePos = {60, 300, 180};

    private boolean mode = false;
    private int index = 0;
    private double distance = 0;
    //Stores position and current index of spindex
    public Spindex(HardwareMap hardwareMap){
        spindex = hardwareMap.get(CRServo.class, "spindex");
        spindexPos = hardwareMap.get(AnalogInput.class, "encoder");
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
    public void lockPos(boolean mode){
        this.mode = mode;
        double targetPos;
        if (!mode){
            targetPos = intakePos[index%3];
        }
        else{
            targetPos = outtakePos[index%3];
        }
        
        double currentPos = getPos();
        distance = currentPos - targetPos;
        
        // Handle wrapping around 360 degrees - find shortest path
        if (distance > 180) {
            distance = distance - 360;
        } else if (distance < -180) {
            distance = distance + 360;
        }

        if (Math.abs(distance) < SpindexValues.range){
            spindex.setPower(0);
        }
        else {
            // Set power with correct direction: negative distance means we're behind, need positive power
            // Positive distance means we're ahead, need negative power
            double power = 0.1;
            if (distance > 0) {
                power = -0.1; // Go backward
            } else {
                power = 0.1; // Go forward
            }
            spindex.setPower(power);
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
