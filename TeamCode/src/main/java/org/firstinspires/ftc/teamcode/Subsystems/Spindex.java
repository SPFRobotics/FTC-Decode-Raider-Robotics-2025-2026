package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Resources.Unit;
//import org.firstinspires.ftc.teamcode.Testing.Test;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.Threshold;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.launchTime;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.maxPower;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.pid;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.tolorence;
import static java.lang.Thread.sleep;

public class Spindex {
    //Servo encoder
    private static AnalogInput spindexPos = null;
    public DcMotorEx spindexMotor = null;
    private ColorFetch colorSensor = null;

    //Stores weather the class is using a motor or servo
    //Stores position and current index of spindex
    private static int index = 0;
    private double threadLoopTime = 0;
    private double currentPos = 0;
    private double error = 0;
    private double offset = 0;
    private final double MAXVOLTAGE = 3.216;
    private boolean outtakeMode = false;
    private boolean autoLoadMode = false;
    private boolean autoLaunchMode = false;
    private boolean terminate = false;
    private boolean atTarget = false;
    private boolean absAndRelInitialized = false;
    private ElapsedTime autoLaunchTimer = new ElapsedTime();

    private char[] slotColorStatus = {'E', 'E', 'E'};
    private boolean[] slotStatus = {false, false, false};
    private boolean ballLatched = false;
    @Config
    public static class SpindexValues{
        public static double maxPower = 1;
        public static double Threshold = 63.75;

        //For abs and rel
        public static double[] pid = {25, 0.05, 0.5};
        public static double tolorence = 5;
        public static double[] intakePos = {2, 122, 242};
        public static double[] outtakePos = {182, 302, 62};

        //Distance/Color sensor
        //Old value is 3.3
        public static double ballDistanceThreshold = 3;
        public static double ballReleaseThreshold = 4.0;
        public static double spindexPowerThreshold = 0.1;
        public static double launchTime = 900;
    }

    //Spindex constructor accepts a boolean. True makes the class use a motor while the input being false makes it use a servo instead
    public Spindex(HardwareMap hardwareMap){
        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexPos = hardwareMap.get(AnalogInput.class, "spindexPos");
        index = 0;
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        offset = AngleUnit.normalizeDegrees(spindexPos.getVoltage()/MAXVOLTAGE*360.0);
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
                setTargetStatus(false);
            }
            else if (Math.abs(error) > tolorence) {
                spindexMotor.setPower(error * kp);
                setTargetStatus(false);

            }
            else {
                spindexMotor.setPower(0);
                setTargetStatus(true);
            }
        }
        else{
            currentPos = spindexPos.getVoltage()/MAXVOLTAGE*360.0;

            error = AngleUnit.normalizeDegrees(target - currentPos);

            double sign = Math.signum(error);

            double kp = maxPower/(Threshold);

            if (Math.abs(error) > Threshold){
                spindexMotor.setPower(maxPower * sign);
                setTargetStatus(false);
            }
            else if (Math.abs(error) > tolorence) {
                spindexMotor.setPower(error * kp);
                setTargetStatus(false);
            }
            else {
                spindexMotor.setPower(0);
                setTargetStatus(true);
            }
        }
    }

    public void initAbsAndRel(){
    }

    //Overloaded method that contains three options in order to maintain compatibility with older programs while adding support for using the abs and relative encoders together.
    /*
    Modes:
    1 - Spindex uses the built-in motor encoder and is controlled with setPower() and the custom made P controller
    2 - Spindex uses the absolute encoder with setPower() and the custom made P controller
    3 - Spindex uses the absolute encoder with the built-in motor encoder. This uses the built-in PID controller FTC provides within their SDK to control the motor.
    */
    public void moveToPos(double target, int mode) {
        if (mode == 1){
            currentPos = AngleUnit.normalizeDegrees((double)spindexMotor.getCurrentPosition()/537.7*360);

            error = AngleUnit.normalizeDegrees(target - currentPos);

            double sign = Math.signum(error);

            double kp = maxPower/Threshold;

            if(Math.abs(error) > Threshold){
                spindexMotor.setPower(maxPower * sign);
                setTargetStatus(false);
            }
            else if (Math.abs(error) > tolorence) {
                spindexMotor.setPower(error * kp);
                setTargetStatus(false);

            }
            else {
                spindexMotor.setPower(0);
                setTargetStatus(true);
            }
        }
        else if (mode == 2){
            currentPos = spindexPos.getVoltage()/MAXVOLTAGE*360.0;

            error = AngleUnit.normalizeDegrees(target - currentPos);

            double sign = Math.signum(error);

            double kp = maxPower/(Threshold);

            if (Math.abs(error) > Threshold){
                spindexMotor.setPower(maxPower * sign);
                setTargetStatus(false);
            }
            else if (Math.abs(error) > tolorence) {
                spindexMotor.setPower(error * kp);
                setTargetStatus(false);
            }
            else {
                spindexMotor.setPower(0);
                setTargetStatus(true);
            }
        }
        else if (mode == 3){
            double relPos = Math.floorMod((int)(((spindexMotor.getCurrentPosition()/537.7*360)+offset+0.5)), 360);
            double error = AngleUnit.normalizeDegrees(target - relPos);
            double ticksError = error/537.7*360;

            spindexMotor.setTargetPosition((int)(spindexMotor.getCurrentPosition()+ticksError+0.5));
            spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(pid[0], pid[1], pid[2]));
            spindexMotor.setPower(1);
        }
        else if (mode == 4){
            double currentPos = getNormEnc(spindexMotor.getCurrentPosition()+(offset/360.0*537.7));
            double error = getNormEnc(target/360.0*537.7 - currentPos);

            spindexMotor.setTargetPosition((int)(spindexMotor.getCurrentPosition()+error+0.5));
            spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(pid[0], pid[1], pid[2]));
            spindexMotor.setPower(1);
        }
        else{
            throw new RuntimeException("The mode of Spindex operation is not an option!");
        }
    }

    public void addIndex(){
        index = Math.floorMod(index+1, 3);
    }

    public void subtractIndex(){
        index = Math.floorMod(index-1, 3);
    }

    public void setIndex(int i){
        index = i;
    }
    public int getIndex(){
        return index;
    }


    public char[] getSlotColorStatus (){
        return slotColorStatus;
    }

    //Methods used to set the color of the balls at the current index, detection is handled in the "main" code
    /*########################################*/
    public void setSlotColorStatus(char color){
        int currentIndex = getIndex();
        if (slotColorStatus[currentIndex] == 'E'){
            slotColorStatus[currentIndex] = color;
        }
    }

    public void clearSlot(int index){
        slotColorStatus[index] = 'E';
    }
    /*########################################*/

    /****Methods to control ball detection, not color****/
    public void addBall(int index){
        slotStatus[index] = true;
    }

    public void clearBall(int index){
        slotStatus[index] = false;
        ballLatched = false;
    }

    public boolean[] getSlotStatus(){
        return slotStatus;
    }

    public void autoLoad(ColorFetch colorSensor){
        double ballDistance = colorSensor.getDistance();

        // Latch releases only when sensor reads far enough (ball has cleared)
        if (ballDistance > SpindexValues.ballReleaseThreshold) {
            ballLatched = false;
        }

        // Detect a new ball only when unlocked, spindex is settled, and sensor reads close
        if (!ballLatched && !getSlotStatus()[getIndex()] && !isOuttakeing() && !isBusy() && ballDistance < SpindexValues.ballDistanceThreshold && colorSensor.getColor() != 0){
            addBall(getIndex());
            ballLatched = true;
        }

        if (isAutoLoading() && slotStatus[getIndex()]) {
            for (int i = 0; i < slotStatus.length; i++) {
                if (!slotStatus[i]) {
                    setIndex(i);
                    break;
                }
            }
        }
    }

    boolean ballFound = false;
    public void autoLaunch(KickerSpindex kicker){
        if (isAutoLaunching() && !slotStatus[getIndex()]) {
            if (!ballFound){
                autoLaunchTimer.reset();
                ballFound = true;
            }
            for (int i = 0; i < slotStatus.length; i++) {
                if (slotStatus[i] && autoLaunchTimer.milliseconds() >= launchTime && getPower() == 0) {
                    setIndex(i);
                    ballFound = false;
                    break;
                }
            }
        }
    }
    /****************************************************/

    public void setTargetStatus(boolean x){
        atTarget = x;
    }

    public void setMode(boolean outtake){
        outtakeMode = outtake;
    }

    public void setPower(int power){
        spindexMotor.setPower(power);
    }

    public void setAutoLoadMode(boolean x){
        autoLoadMode = x;
    }

    public void setAutoLaunchMode(boolean x){
        autoLaunchMode = x;
    }

    public double getPos(){
        return getVoltage()/MAXVOLTAGE*360.0;
    }

    public double getEncPos(){
        return AngleUnit.normalizeDegrees((spindexMotor.getCurrentPosition()/537.7*360.0)+offset);
    }

    public double getNormEnc(double encCounts){
        return (encCounts + (537.7/2)) % 537.7 - (537.7/2);
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

    public boolean isAutoLoading(){
        return autoLoadMode;
    }

    public boolean isAutoLaunching(){
        return autoLaunchMode;
    }

    public boolean atTarget(){
        return Math.abs(error) <= tolorence;
    }

    public boolean atTarget(int tolorence){
        return Math.abs(error) <= tolorence;
    }

    public boolean isBusy(){
        return spindexMotor.isBusy();
    }
}