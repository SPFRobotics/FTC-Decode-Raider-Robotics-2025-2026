package org.firstinspires.ftc.teamcode.Game.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Resources.Unit;
import org.firstinspires.ftc.teamcode.Testing.Test;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.Threshold;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.maxPower;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.tolorence;

public class Spindex {
    //Servo encoder
    private static AnalogInput spindexPos = null;
    public DcMotorEx spindexMotor = null;
    //Stores weather the class is using a motor or servo
    //Stores position and current index of spindex
    private static int index = 0;
    private double threadLoopTime = 0;
    private double currentPos = 0;
    private double error = 0;
    private boolean outtakeMode = false;
    private boolean terminate = false;

    private static char[] slotStatus = {'E', 'E', 'E'};
    @Config
    public static class SpindexValues{
        public static double maxPower = 1;
        public static double Threshold = 150;
        public static double tolorence = 3;
        public static double[] intakePos = {0, 120, 240};
        public static double[] outtakePos = {60, 180, 300};
        public static double ballDistanceThreshold = 2;
        public static double slotTimer = 500;
    }

    //Spindex constructor accepts a boolean. True makes the class use a motor while the input being false makes it use a servo instead
    public Spindex(HardwareMap hardwareMap){
        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexPos = hardwareMap.get(AnalogInput.class, "spindexPos");
        slotStatus = new char[]{'E', 'E', 'E'};
        index = 0;
    }


    //Moves the servo or motor to the target position by finding the shortest path
    public void moveToPos(double target, boolean absEncoder) {
        if (!absEncoder){
            currentPos = AngleUnit.normalizeDegrees((double)spindexMotor.getCurrentPosition()/537.7*360);

            error = AngleUnit.normalizeDegrees(target - currentPos);

            double sign = Math.signum(error);

            double kp = maxPower/Threshold;

            if(Math.abs(error) > Threshold){
                spindexMotor.setPower(maxPower * sign);
            }
            else if (Math.abs(error) > tolorence) {
                spindexMotor.setPower(error * kp);

            }
            else {
                spindexMotor.setPower(0);
            }
        }
        else{
            currentPos = spindexPos.getVoltage()/3.3*360.0;

            error = AngleUnit.normalizeDegrees(target - currentPos);

            double sign = Math.signum(error);

            double tolorence = 5;

            double kp = maxPower/Threshold;

            if(Math.abs(error) > Threshold){

                spindexMotor.setPower(maxPower * sign);

            } else if (Math.abs(error) > tolorence) {

                spindexMotor.setPower(error * kp);

            }else {
                spindexMotor.setPower(0);
            }
        }
    }

    public void addIndex(){
        index++;
        index = Math.floorMod(index, 3);
    }

    public void subtractIndex(){
        index--;
        index = Math.floorMod(index, 3);
    }

    public void setIndex(int i){
        index = i;
    }
    public int getIndex(){
        return index;
    }

    public char[] getSlotStatus (){
        return slotStatus;
    }

    //Methods used to set the color of the balls at the current index, detection is handled in the "main" code
    /*########################################*/
    public void setSlotStatus(char color){
        int currentIndex = getIndex();
        if (slotStatus[currentIndex] == 'E'){
            slotStatus[currentIndex] = color;
        }
    }

    public void clearSlot(int index){
        slotStatus[index] = 'E';
    }
    /*########################################*/

    public void setMode(boolean outtake){
        outtakeMode = outtake;
    }

    public double getPos(){
        return currentPos;
    }

    public void storeThreadLoopTime(double milliseconds){
        threadLoopTime = milliseconds;
    }

    public double getThreadLoopTime(){
        return threadLoopTime;
    }

    public void exitProgram(){
        terminate = true;
    }

    public boolean getProgramState(){
        return terminate;
    }

    public double getVoltage(){
        return spindexPos.getVoltage();
    }

    public double getPower(){
        return spindexMotor.getPower();
    }

    public double getError(){
        return error;
    }

    public double getAmps(){
        return spindexMotor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isOuttakeing(){
        return outtakeMode;
    }
}