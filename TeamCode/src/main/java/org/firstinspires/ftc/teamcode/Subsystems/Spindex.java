package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Resources.Unit;
//import org.firstinspires.ftc.teamcode.Testing.Test;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.Threshold;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.launchTime;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.maxPower;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.pidf;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.tolorence;
import static java.lang.Thread.sleep;

public class Spindex {
    //Servo encoder
    private static AnalogInput spindexPos = null;
    public DcMotorEx spindexMotor = null;
    private ColorFetch colorSensor = null;

    //Stores weather the class is using a motor or servo
    //Stores position and current index of spindex
    private int index = 0;
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
    private ElapsedTime autoLaunchTimer = new ElapsedTime();

    private char[] slotColors = {'E', 'E', 'E'};
    private boolean[] slotStatus = {false, false, false};
    private boolean ballLatched = false;

    private enum AutoSortState { FIND_NEXT, ROTATING, LAUNCHING, COMPLETE }
    private AutoSortState autoSortState = AutoSortState.FIND_NEXT;
    private int sortPatternIndex = 0;
    private boolean autoSortActive = false;

    public  String motif21Pattern = "GPP";
    public  String motif22Pattern = "PGP";
    public  String motif23Pattern = "PPG";

    @Config
    public static class SpindexValues{
        public static double maxPower = 1;
        public static double Threshold = 63.75;

        public static double[] pidf = {35, 0.3, 12, 0};
        public static double tolorence = 5;
        public static double[] intakePos = {2, 122, 242};
        public static double[] outtakePos = {182, 302, 62};

        //Distance/Color sensor
        //Old value is 3.3
        public static double ballDistanceThreshold = 3;
        public static double ballReleaseThreshold = 4.0;
        public static double launchTime = 900;

        // Motif patterns: each string is the launch order of colors (P=purple, G=green)
        // These build the pattern on the ramp from bottom to top

    }

    //Spindex constructor accepts a boolean. True makes the class use a motor while the input being false makes it use a servo instead
    public Spindex(HardwareMap hardwareMap){
        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        spindexPos = hardwareMap.get(AnalogInput.class, "spindexPos");

        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        offset = AngleUnit.normalizeDegrees(spindexPos.getVoltage()/MAXVOLTAGE*360.0);
        index = 0;
    }

    //Overloaded method that contains three options in order to maintain compatibility with older programs while adding support for using the abs and relative encoders together.
    /*
    Modes:
    1 - Spindex uses the built-in motor encoder and is controlled with setPower() and the custom made P controller
    2 - Spindex uses the absolute encoder with setPower() and the custom made P controller
    3 - Removed
    4 - Spindex uses the absolute encoder with the built-in motor encoder. This uses the built-in PID controller FTC provides within their SDK to control the motor.
    */
    public void moveToPos(double target, int mode) {
        double sign = 0;
        double kp = 0;
        mode = mode == 3 ? 4 : mode;
        switch (mode) {
            case 1:
                currentPos = AngleUnit.normalizeDegrees((double) spindexMotor.getCurrentPosition() / 537.7 * 360);

                error = AngleUnit.normalizeDegrees(target - currentPos);

                sign = Math.signum(error);

                kp = maxPower / Threshold;

                if (Math.abs(error) > Threshold) {
                    spindexMotor.setPower(maxPower * sign);
                    setTargetStatus(false);
                } else if (Math.abs(error) > tolorence) {
                    spindexMotor.setPower(error * kp);
                    setTargetStatus(false);

                } else {
                    spindexMotor.setPower(0);
                    setTargetStatus(true);
                }
                break;
            case 2:
                currentPos = spindexPos.getVoltage() / MAXVOLTAGE * 360.0;

                error = AngleUnit.normalizeDegrees(target - currentPos);

                sign = Math.signum(error);

                kp = maxPower / (Threshold);

                if (Math.abs(error) > Threshold) {
                    spindexMotor.setPower(maxPower * sign);
                    setTargetStatus(false);
                } else if (Math.abs(error) > tolorence) {
                    spindexMotor.setPower(error * kp);
                    setTargetStatus(false);
                } else {
                    spindexMotor.setPower(0);
                    setTargetStatus(true);
                }
                break;
            case 4:
                double currentPos = getNormEnc(spindexMotor.getCurrentPosition() + (offset / 360.0 * 537.7));
                double error = getNormEnc(target / 360.0 * 537.7 - currentPos);

                spindexMotor.setTargetPosition((int) (spindexMotor.getCurrentPosition() + error + 0.5));
                spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spindexMotor.setVelocityPIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]);
                spindexMotor.setPower(1);
                break;
            default:
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


    public char[] getSlotColors (){
        return slotColors;
    }

    //Methods used to set the color of the balls at the current index, detection is handled in the "main" code
    /*########################################*/
    public void setSlotColor(char color){
        int currentIndex = getIndex();
        if (slotColors[currentIndex] == 'E'){
            slotColors[currentIndex] = color;
        }
    }

    public void clearColor(int index){
        slotColors[index] = 'E';
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
        if (!getSlotStatus()[getIndex()] && !isOuttakeing() && !isBusy() && ballDistance < SpindexValues.ballDistanceThreshold){
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

    public void autoSort(Outtake outtake, int motifId) {
        if (!autoSortActive || motifId < 21 || motifId > 23) return;

        String patternStr;
        switch (motifId) {
            case 21: patternStr = motif21Pattern; break;
            case 22: patternStr = motif22Pattern; break;
            case 23: patternStr = motif23Pattern; break;
            default: return;
        }
        char[] pattern = patternStr.toCharArray();

        switch (autoSortState) {
            case FIND_NEXT:
                if (sortPatternIndex >= pattern.length) {
                    autoSortState = AutoSortState.COMPLETE;
                    return;
                }
                char needed = pattern[sortPatternIndex];
                for (int i = 0; i < 3; i++) {
                    if (slotStatus[i] && slotColors[i] == needed) {
                        setIndex(i);
                        setMode(true);
                        autoSortState = AutoSortState.ROTATING;
                        return;
                    }
                }
                break;

            case ROTATING:
                double targetRPM = outtake.isFarLocation()
                        ? Outtake.OuttakeConfig.farRPM
                        : Outtake.OuttakeConfig.closeRPM;
                outtake.setRPM(targetRPM);
                moveToPos(SpindexValues.outtakePos[getIndex()], 4);
                if (!isBusy()) {
                    outtake.resetKickerCycle();
                    autoSortState = AutoSortState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                double rpm = outtake.isFarLocation()
                        ? Outtake.OuttakeConfig.farRPM
                        : Outtake.OuttakeConfig.closeRPM;
                moveToPos(SpindexValues.outtakePos[getIndex()], 4);
                outtake.enableSpindexKickerCycle(true, rpm);
                if (outtake.getKickerCycleCount() >= 1) {
                    clearBall(getIndex());
                    clearColor(getIndex());
                    sortPatternIndex++;
                    outtake.resetKickerCycle();
                    autoSortState = AutoSortState.FIND_NEXT;
                }
                break;

            case COMPLETE:
                autoSortActive = false;
                break;
        }
    }

    public void setAutoSortActive(boolean active) {
        autoSortActive = active;
        if (active) {
            sortPatternIndex = 0;
            autoSortState = AutoSortState.FIND_NEXT;
        }
    }

    public void resetAutoSort() {
        autoSortActive = false;
        sortPatternIndex = 0;
        autoSortState = AutoSortState.FIND_NEXT;
    }

    public boolean isAutoSorting() {
        return autoSortActive;
    }

    public boolean isAutoSortComplete() {
        return autoSortState == AutoSortState.COMPLETE;
    }

    public String getAutoSortStateName() {
        return autoSortState.name();
    }

    public int getSortPatternIndex() {
        return sortPatternIndex;
    }

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

    public double getNormAngPos(){
        return AngleUnit.normalizeDegrees((spindexMotor.getCurrentPosition()/537.7*360.0)+offset);
    }

    public double getNormEnc(double encCounts){
        return (encCounts + (537.7/2)) % 537.7 - (537.7/2);
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

    public void showTelemetry(Telemetry telemetry){
        telemetry.addLine("------------------------------------------------------------------------------------");
        telemetry.addLine("Spindex");
        telemetry.addLine("Spindex Encoder Count: " + spindexMotor.getCurrentPosition());
        telemetry.addLine("Spindex Wrapped Encoder Position: " + getNormEnc(spindexMotor.getCurrentPosition()));
        telemetry.addLine("Spindex Angular Position: " + ((spindexMotor.getCurrentPosition()/537.7*360.0)+offset));
        telemetry.addLine("Spindex Normalized Angular Position: " + getNormAngPos());
        telemetry.addLine("Spindex Absolute Encoder Position: " + getPos());
        telemetry.addLine("Spindex Absolute Encoder Voltage: " + getVoltage());
        telemetry.addLine("------------------------------------------------------------------------------------");
    }

    public void showTelemetry(MultipleTelemetry telemetry){
        telemetry.addLine("------------------------------------------------------------------------------------");
        telemetry.addLine("Spindex");
        telemetry.addLine("Spindex Encoder Count: " + spindexMotor.getCurrentPosition());
        telemetry.addLine("Spindex Wrapped Encoder Position: " + getNormEnc(spindexMotor.getCurrentPosition()));
        telemetry.addLine("Spindex Angular Position: " + (((spindexMotor.getCurrentPosition()/537.7*360.0)+offset)%360));
        telemetry.addLine("Spindex Normalized Angular Position: " + getNormAngPos());
        telemetry.addLine("Spindex Absolute Encoder Position: " + getPos());
        telemetry.addLine("Spindex Absolute Encoder Voltage: " + getVoltage());
        telemetry.addLine("------------------------------------------------------------------------------------");
    }
}