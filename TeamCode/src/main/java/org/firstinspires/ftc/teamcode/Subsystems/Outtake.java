package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.*;

public class Outtake {
    @Config
    public static class OuttakeConfig{
        public static double farRPM = 3200;
        public static double closeRPM = 2700;
        public static double sortRPM = 1000;
        public static double p = 268;
        public static double i = 14.99;
        public static double d = 0;
        public static double f = 14.99;
        public static double gearRatio = 1.0625;

        public static double kickerWaitTIme = 2;
        public static double kickerSettleTime = 0.2;
    }

    private ColorFinder colorFinder = null;
    public ColorSensor hardwareColorSensor = null;
    public DcMotorEx outtakeMotor = null;
    private KickerGrav kickerGrav = null;

    private KickerSpindex kickerSpindex = null;
    private boolean isActive = false;
    public boolean launched = false;

    public Limelight limelight = null;
    private int encoderCount = 0;
    private boolean isFarLocation = true; // true = far (77%), false = short (55%)
    private boolean isBoosted = false; // Track if we're currently boosting
    private ElapsedTime boostTimer = new ElapsedTime(); // Timer for boost duration
    private int updateCounter = 0; // Counter for RPM checking interval
    private double lastRPM = 0; // Store last RPM reading
    private int kickerCycleCount = 0;
    private enum KickerCycleState { IDLE, ENSURE_DOWN, DOWN_SETTLE, WAIT_FOR_RPM, MOVING_UP, MOVING_DOWN }
    private KickerCycleState kickerState = KickerCycleState.IDLE;
    private ElapsedTime kickerStateTimer = new ElapsedTime();

    //The "E"ncoder "R"esolution our current motor runs at.
    int motorER = 28;

    private ElapsedTime clock = new ElapsedTime();
    //Interval in seconds of outtake cycle
    private ElapsedTime interval = new ElapsedTime();
    // Constructor - initializes the intake motor


    public Outtake(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        outtakeMotor.setVelocityPIDFCoefficients(p, i, d, f);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kickerGrav = new KickerGrav(hardwareMap);
        kickerSpindex = new KickerSpindex(hardwareMap);
        //limelight = new Limelight(hardwareMap);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Constructor for spindex-only (no KickerGrav servo needed)
    public Outtake(HardwareMap hardwareMap, boolean useSpindexOnly) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setVelocityPIDFCoefficients(p, i, d, f);
        if (useSpindexOnly) {
            kickerSpindex = new KickerSpindex(hardwareMap);
            kickerGrav = null;
        } else {
            kickerGrav = new KickerGrav(hardwareMap);
        }
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*public void ColorSort(){

        limelight.getMotifId();
        LLResult result = limelight.getLatestResult();
        setRPM(Outtake.OuttakeSpeed.sortRPM);


        if(result.equals(22)&& colorFinder.isGreen()&&kickerCycleCount==1||kickerCycleCount==3){
            enableKickerCycle(true, OuttakeSpeed.sortRPM);
            kicker.down(true);

        }
    }*/

    // Switch between far and short locations
    public void switchLocation() {
        isFarLocation = !isFarLocation;
    }

    // Get current location (true = far, false = short)
    public boolean isFarLocation() {
        return isFarLocation;
    }

    // Get current location name
    public String getLocationName() {
        return isFarLocation ? "FAR" : "SHORT";
    }

    // Getter for active state
    public boolean isActive() {
        return isActive;
    }

    // Manual control methods (optional - for testing or manual override)
    public void activate() {
        isActive = true;
    }

    public void deactivate() {
        isActive = false;
    }

    //This returns the speed of the flywheel NOT the motor itself
    public double getRPM() {
        return ((outtakeMotor.getVelocity()*60)/28/gearRatio);
    }


    public void enableKickerCycle(boolean x, double RPM){
        double time = interval.seconds();
        if (x){
            if (time >= 1 && time < 2 && getRPM() >= RPM-500){
                kickerGrav.up();
                launched = true;
            }
            else if (time >= 2){
                kickerGrav.down();
                if (launched){
                    kickerCycleCount++;
                }
                launched = false;
                interval.reset();
            }
        }
        else{
            kickerGrav.up();
            interval.reset();
        }
    }


    public void enableSpindexKickerCycle(boolean x, double RPM){
        double time = interval.seconds();
        if (x){
            // Phase 1: Wait for flywheel RPM, then kick up and start the timer
            if (!launched && getRPM() >= RPM - 100){
                kickerSpindex.up();
                launched = true;
                interval.reset(); // Timer starts when kick actually begins
            }
            // Phase 2: After a full 200ms of kick travel, bring it back down
            else if (launched && time >= .3){
                kickerSpindex.down();
                kickerCycleCount++;
                launched = false;
                interval.reset();
            }
        }
        else{
            kickerSpindex.up();
            interval.reset();
        }
    }

    public int getKickerCycleCount(){
        return kickerCycleCount;
    }

    public String getKickerState(){
        return kickerState.name();
    }

    public void resetKickerCycle(){
        kickerCycleCount = 0;
        launched = false;
        interval.reset();
        kickerStateTimer.reset();
        kickerState = KickerCycleState.IDLE;
        if (kickerGrav != null){
            kickerGrav.down();
        }
        if (kickerSpindex != null){
            kickerSpindex.down();
        }
    }

    public double getCurrentCycleTime(){
        return interval.seconds();
    }

    public void shortAuto(){
        double RPM = 2700;


        setRPM(RPM);
        enableKickerCycle(true,RPM);


    }

    public double getPower(){
        return outtakeMotor.getPower();
    }

    //This sets the speed of the flywheel to the correct RPM NOT the motor itself
    public void setRPM(double rpm){
        double tps = (rpm / 60.0) * 28;
        outtakeMotor.setVelocity(tps*gearRatio);
    }

    public double getInverval(){
        return interval.seconds();
    }
}