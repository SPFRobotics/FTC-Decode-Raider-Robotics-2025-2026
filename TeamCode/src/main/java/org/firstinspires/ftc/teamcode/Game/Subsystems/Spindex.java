package org.firstinspires.ftc.teamcode.Game.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Testing.Test;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.Threshold;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.maxPower;

public class Spindex {
    //Servo encoder
    private static AnalogInput spindexPos = null;
    private DcMotor spindexMotor = null;
    private CRServo spindexServo = null;
    //Stores weather the spindex is an intake or outtake mode
    private boolean mode = false;
    //Stores weather the class is using a motor or servo
    private boolean motor = false;
    //Stores position and current index of spindex
    private int index = 0;
    private double currentPos = 0;
    @Config
    public static class SpindexValues{
        public static double maxPower = 0.6;
        public static double Threshold = 150;
        public static double[] intakePos = {0, 120, 240};
        public static double[] outtakePos = {60, 180, 300};
    }

    //Spindex constructor accepts a boolean. True makes the class use a motor while the input being false makes it use a servo instead
    public Spindex(HardwareMap hardwareMap, boolean motor){
        this.motor = motor;
        if (motor){
            spindexMotor = hardwareMap.get(DcMotor.class, "spindex");
            spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{
            spindexServo = hardwareMap.get(CRServo.class, "spindex");
            spindexPos = hardwareMap.get(AnalogInput.class, "spindexPos");
            spindexServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }


    //Moves the servo or motor to the target position by finding the shortest path
    public void moveToPos(double target) {
        if (motor){
            currentPos = Math.floorMod((int)(spindexMotor.getCurrentPosition()/537.7*360), 360);

            double error = AngleUnit.normalizeDegrees(target - currentPos);

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
        else{
            currentPos = (int)((spindexPos.getVoltage())/3.3*360);

            double error = AngleUnit.normalizeDegrees(target - currentPos);

            double sign = Math.signum(error);

            double tolorence = 5;

            double kp = maxPower/Threshold;

            if(Math.abs(error) > Threshold){

                spindexServo.setPower(maxPower * sign);

            } else if (Math.abs(error) > tolorence) {

                spindexServo.setPower(error * kp);

            }else {
                spindexServo.setPower(0);
            }
        }
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

    /*
    private double getMinDistance(double[] positions){
        double diff = positions[((index%3)+3)%3]-getPos();
        return ((diff+540)%360)-180;
    }



     */
/*
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
            this.mode = true;
        }
        else{
            spindex.setPower(Math.min((getMinDistance(outtakePos)/SpindexValues.p) * speed, 1));
            this.mode = false;
        }
    }
 */
    public boolean isIntake(){
        return mode;
    }

    /*public void zero(){

    }*/

    public void setPower(double power){
        if (motor){
            spindexMotor.setPower(power);
        }
        else{
            spindexServo.setPower(power);
        }
    }

    public double getPos(){
        return currentPos;
    }
}