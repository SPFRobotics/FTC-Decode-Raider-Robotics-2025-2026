package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.Arrays;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.Testing.Test;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.ballDistanceThreshold;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.pidf;
import static java.lang.Thread.sleep;

public class Spindex {
    //Servo encoder
    private static AnalogInput spindexPos = null;
    public DcMotorEx spindexMotor = null;
    private ColorFetch colorSensor = null;

    //Stores weather the class is using a motor or servo
    //Stores position and current index of spindex
    private int index = 0;

    double encoderTicks = 537.7;
    private double threadLoopTime = 0;
    private double currentPos = 0;
    //Error in ticks
    double error = 0;
    double targetPos = 0;
    protected double offset = 1;
    private final double MAXVOLTAGE = 3.216;
    private boolean outtakeMode = false;
    private boolean autoLoadMode = false;
    private boolean autoLaunchMode = false;
    private boolean terminate = false;
    private boolean atTarget = false;
    private ElapsedTime autoLaunchTimer = new ElapsedTime();

    private char[] slotColors = {'E', 'E', 'E'};

    private enum AutoSortState { FIND_NEXT, ROTATING, LAUNCHING, TRY_SHOOT_UNDETECTED, COMPLETE }
    private AutoSortState autoSortState = AutoSortState.FIND_NEXT;
    private int sortPatternIndex = 0;
    private boolean autoSortActive = false;
    private int tryShootSlot = 0;
    private boolean tryingUndetected = false;
    private ElapsedTime tryShootTimer = new ElapsedTime();
    private ElapsedTime rotateSettleTimer = new ElapsedTime();
    private static final long TRY_SHOOT_TIMEOUT_MS = 400;

    public static final String motif21Pattern = "GPP";
    public static final String motif22Pattern = "PGP";
    public static final String motif23Pattern = "PPG";

    @Config
    public static class SpindexValues{
        //public static double maxPower = 1;
        //public static double Threshold = 63.75;

        public static double[] pidf = {35, 0.3, 12, 0};
        public static double offset = 10;
        public static double[] intakePos = {0+offset, 120+offset, 240+offset};
        public static double[] outtakePos = {180+offset, 300+offset, 60+offset};

        //Distance/Color sensor
        //Old value is 3.3
       public static double ballDistanceThreshold = 2.3;
        //public static double ballReleaseThreshold = 4.0;
       // public static double launchTime = 900;

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
    3 - Removed, defaults to mode 4 if selected
    4 - Spindex uses the absolute encoder with the built-in motor encoder. This uses the built-in PID controller FTC provides within their SDK to control the motor.
    */
    public void moveToPos(double target) {
        double currentPos = getNormEnc(spindexMotor.getCurrentPosition() + (offset / 360.0 * encoderTicks));
        error = getNormEnc(target / 360.0 * encoderTicks - currentPos);
        targetPos = (spindexMotor.getCurrentPosition() + error + 0.5);

        spindexMotor.setTargetPosition((int)targetPos);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setVelocityPIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]);
        spindexMotor.setPower(1);
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

    public void addToTarget(double x){
        targetPos += x;
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

    public void clearBall(int index){
        slotColors[index] = 'E';
    }

    public int getIndexOfColor(char color){
        int index = 0;
        for (int i = 0; i < 3; i++){
            if (color == slotColors[i]){
                index = i;
                break;
            }
            else{
                index = -1;
            }
        }
        return index;
    }
    /*########################################*/

    public void autoLoad(ColorFetch colorSensor){
        double ballDistance = colorSensor.getDistance();

        // Detect a new ball only when unlocked, spindex is settled, and sensor reads close
        if (getSlotColors()[getIndex()] == 'E' && !isOuttakeing() && !isBusy() && ballDistance < SpindexValues.ballDistanceThreshold){
            setSlotColor(colorSensor.getAverageColor());
        }

        if (isAutoLoading() && slotColors[getIndex()] != 'E') {
            for (int i = 0; i < slotColors.length; i++) {
                if (slotColors[i] == 'E') {
                    setIndex(i);
                    break;
                }
            }
        }
    }

    //Overloaded to use both color sensors and picking the one with a valid reading
    public void autoLoad(DualColorFetch colorSensor){
        double[] ballDistances = colorSensor.getDistances();
        float currentHue = 0;

        //currentColor is set to the distance sensor which reads a distance under the threshold first
        if (ballDistances[0] <= ballDistanceThreshold){
            currentHue = colorSensor.getHues()[0];
        }
        if (ballDistances[1] <= ballDistanceThreshold){
            currentHue = colorSensor.getHues()[1];
        }

        if (currentHue != 0 && getSlotColors()[getIndex()] == 'E' && !isBusy()){
            setSlotColor(colorSensor.getColor(currentHue));
        }

        if (isAutoLoading() && slotColors[getIndex()] != 'E') {
            for (int i = 0; i < slotColors.length; i++) {
                if (slotColors[i] == 'E') {
                    setIndex(i);
                    break;
                }
            }
        }
    }
    public void autoLoadByDistance(DualColorFetch colorSensor) {
        if (getSlotColors()[getIndex()] == 'E' && !isBusy() && colorSensor.ballDetected()) {
            slotColors[getIndex()] = 'U';
        }

        if (isAutoLoading() && slotColors[getIndex()] != 'E') {
            for (int i = 0; i < slotColors.length; i++) {
                if (slotColors[i] == 'E') {
                    setIndex(i);
                    break;
                }
            }
        }
    }

    enum AutoLaunchState {
        NEXT_SLOT,
        WAITFORSPINDEX,
        LAUNCH
    }
    AutoLaunchState autoLaunchState = AutoLaunchState.NEXT_SLOT;
    ElapsedTime kickerTimer = new ElapsedTime();
    int autoLaunchCount = 0;

    public void autoLaunch(KickerSpindex kicker){
        if (!autoLaunchMode) return;

        switch (autoLaunchState){
            case NEXT_SLOT:
                if (autoLaunchCount >= 3) {
                    autoLaunchMode = false;
                    return;
                }
                autoLaunchState = AutoLaunchState.WAITFORSPINDEX;
                break;

            case WAITFORSPINDEX:
                kicker.down();
                if (!isBusy()){
                    autoLaunchState = AutoLaunchState.LAUNCH;
                    kickerTimer.reset();
                }
                break;

            case LAUNCH:
                if (kickerTimer.milliseconds() < 100) {
                    kicker.up();
                } else if (kickerTimer.milliseconds() < 300) {
                    kicker.down();
                } else {
                    clearBall(index);
                    addIndex();
                    autoLaunchCount++;
                    autoLaunchState = AutoLaunchState.NEXT_SLOT;
                }
                break;
        }
    }

    public void resetAutoLaunch() {
        autoLaunchState = AutoLaunchState.NEXT_SLOT;
        autoLaunchCount = 0;
    }



    public void autoSort(Outtake outtake, int motifId, String knownSlotColors) {
        autoSort(outtake, motifId, null, knownSlotColors);
    }

    public void autoSort(Outtake outtake, int motifId, Turret turret, String knownSlotColors) {
        if (autoSortState == AutoSortState.FIND_NEXT && sortPatternIndex == 0) {
            for (int i = 0; i < 3 && i < knownSlotColors.length(); i++) {
                slotColors[i] = knownSlotColors.charAt(i);
            }
        }
        autoSort(outtake, motifId, turret);
    }

    public void autoSort(Outtake outtake, int motifId, Turret turret) {
        if (!autoSortActive) return;
        if (motifId < 21 || motifId > 23) motifId = 21;

        String patternStr;
        switch (motifId) {
            case 22: patternStr = motif22Pattern; break;
            case 23: patternStr = motif23Pattern; break;
            default: patternStr = motif21Pattern; break;
        }
        char[] pattern = patternStr.toCharArray();
        System.out.printf("autoSort: target pattern- %s%n",patternStr);
        System.out.printf("autosort: slotColors %s%n", slotColors.toString());
        System.out.printf("autoSort:sortPatternIndex %d%n",sortPatternIndex);

        switch (autoSortState) {
            case FIND_NEXT:
                System.out.printf("autoSort: FIND_NEXT-sortPatternIndex%n");
                if (sortPatternIndex >= pattern.length) {
                    autoSortState = AutoSortState.COMPLETE;
                    System.out.printf("autoSort: FIND_NEXT-COMPLETE%n");
                    return;
                }

                boolean allFilled = slotColors[0] != 'E' && slotColors[1] != 'E' && slotColors[2] != 'E';
                if (!allFilled && sortPatternIndex == 0) {
                    tryShootSlot = 0;
                    tryingUndetected = true;
                    autoSortState = AutoSortState.TRY_SHOOT_UNDETECTED;
                    return;
                }

                char needed = pattern[sortPatternIndex];
                for (int i = 0; i < 3; i++) {
                    if (slotColors[i] == needed) {
                        setMode(true);
                        setIndex(i);
                        rotateSettleTimer.reset();
                        autoSortState = AutoSortState.ROTATING;
                        return;
                    }
                }
                for (int i = 0; i < 3; i++) {
                    if (slotColors[i] != 'E') {
                        setMode(true);
                        setIndex(i);
                        rotateSettleTimer.reset();
                        autoSortState = AutoSortState.ROTATING;
                        System.out.printf("autoSort: FIND_NEXT-Found needed %c,%d%n",
                                needed,i);
                        return;
                    }
                }
                System.out.printf("autoSort: FIND_NEXT-No artifacts found%n");
                sortPatternIndex++;
                break;

            case TRY_SHOOT_UNDETECTED:
                if (tryShootSlot >= 3) {
                    autoSortState = AutoSortState.COMPLETE;
                    return;
                }
                setMode(true);
                setIndex(tryShootSlot);
                rotateSettleTimer.reset();
                autoSortState = AutoSortState.ROTATING;
                break;

            case ROTATING:
                double targetRPM = outtake.isFarLocation()
                        ? Outtake.OuttakeConfig.farRPM
                        : Outtake.OuttakeConfig.closeRPM;
                outtake.setRPM(targetRPM);
                if (!isBusy() && rotateSettleTimer.milliseconds() > 200) {
                    autoSortState = AutoSortState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                targetRPM = outtake.isFarLocation()
                        ? Outtake.OuttakeConfig.farRPM
                        : Outtake.OuttakeConfig.closeRPM;
                if (turret == null || turret.isTurretAtTarget()) {
                    outtake.enableSpindexKickerCycle(true, targetRPM);
                    System.out.printf("autoSort: ROTATING-Launching %n");
                }
                if (outtake.getKickerCycleCount() >= 1) {
                    clearBall(getIndex());
                    outtake.resetKickerCycle();
                    if (tryingUndetected) {
                        tryShootSlot++;
                        autoSortState = AutoSortState.TRY_SHOOT_UNDETECTED;
                    } else {
                        sortPatternIndex++;
                        autoSortState = AutoSortState.FIND_NEXT;
                        System.out.printf("autoSort: LAUNCHING-Kicker>=1%n");
                    }
                }
                break;

            case COMPLETE:
                autoSortActive = false;
                System.out.printf("autoSort: COMPLETE%n");
                break;
        }
    }

    public void setAutoSortActive(boolean active) {
        autoSortActive = active;
        if (active) {
            sortPatternIndex = 0;
            tryingUndetected = false;
            autoSortState = AutoSortState.FIND_NEXT;
        }
    }

    public void resetAutoSort() {
        autoSortActive = false;
        sortPatternIndex = 0;
        tryingUndetected = false;
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

    public void setPower(double power){
        spindexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        return AngleUnit.normalizeDegrees((spindexMotor.getCurrentPosition()/encoderTicks*360.0)+offset);
    }

    public double getNormEnc(double encCounts){
        return (encCounts + (encoderTicks/2)) % encoderTicks - (encoderTicks/2);
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

    public void setError(double error){
        this.error = error;
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

    public boolean atTarget(int tolorence){
        return Math.abs(error) <= tolorence;
    }

    public boolean isBusy(){
        return spindexMotor.isBusy();
    }

    public double getVelocity(){
        return spindexMotor.getVelocity();
    }

    //Telemetry Blocks
    public void showTelemetry(Telemetry telemetry){
        telemetry.addLine("------------------------------------------------------------------------------------");
        telemetry.addLine("Spindex");
        telemetry.addLine("Spindex Encoder Count: " + spindexMotor.getCurrentPosition());
        telemetry.addLine("Spindex Wrapped Encoder Position: " + getNormEnc(spindexMotor.getCurrentPosition()));
        telemetry.addLine("Spindex Angular Position: " + ((spindexMotor.getCurrentPosition()/encoderTicks*360.0)+offset));
        telemetry.addLine("Spindex Normalized Angular Position: " + getNormAngPos());
        telemetry.addLine("Spindex Absolute Encoder Position: " + getPos());
        telemetry.addLine("Spindex Absolute Encoder Voltage: " + getVoltage());
        telemetry.addLine("Slot Colors: "  + Arrays.toString(getSlotColors()));
        telemetry.addLine("------------------------------------------------------------------------------------");
    }

    public void showTelemetry(MultipleTelemetry telemetry){
        telemetry.addLine("------------------------------------------------------------------------------------");
        telemetry.addLine("Spindex");
        telemetry.addLine("Spindex Encoder Count: " + spindexMotor.getCurrentPosition());
        telemetry.addLine("Spindex Wrapped Encoder Position: " + getNormEnc(spindexMotor.getCurrentPosition()));
        telemetry.addLine("Spindex Angular Position: " + ((spindexMotor.getCurrentPosition()/encoderTicks*360.0)+offset));
        telemetry.addLine("Spindex Normalized Angular Position: " + getNormAngPos());
        telemetry.addLine("Spindex Absolute Encoder Position: " + getPos());
        telemetry.addLine("Spindex Absolute Encoder Voltage: " + getVoltage());
        telemetry.addLine("Slot Colors: "  + Arrays.toString(getSlotColors()));
        telemetry.addLine("------------------------------------------------------------------------------------");
    }
}