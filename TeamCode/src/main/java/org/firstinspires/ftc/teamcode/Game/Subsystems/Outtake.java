package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake.OuttakeConfig.*;

public class Outtake {
    @Config
    public static class OuttakeConfig{
        public static double farRPM = 3200;
        public static double closeRPM = 2700;
        public static double sortRPM = 1000;
        public static double p = 100;
        public static double i = 3;
        public static double d = 0;
        public static double f = 0;
        public static double gearRatio = 1.0625;

        public static double kickerWaitTIme = 2;
        public static double kickerSettleTime = 0.2;
    }

    private ColorFinder colorFinder = null;
    public ColorSensor hardwareColorSensor = null;
    public DcMotorEx outtakeMotor = null;
    private KickerGrav kickerGrav = null;
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
        //outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setVelocityPIDFCoefficients(p, i, d, f);
        kickerGrav = new KickerGrav(hardwareMap);
        //limelight = new Limelight(hardwareMap);
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
        if (!x){
            resetKickerCycle();
            kickerGrav.up();
            return;
        }

        if (kickerState == KickerCycleState.IDLE){
            kickerState = KickerCycleState.ENSURE_DOWN;
            kickerStateTimer.reset();
            interval.reset();
            kickerGrav.down();
        }

        switch (kickerState){
            case ENSURE_DOWN:
                // Make sure we are at the down encoder position before starting the shot cycle
                kickerGrav.down();
                if (kickerGrav.isAtDownPosition()){
                    kickerState = KickerCycleState.DOWN_SETTLE;
                    kickerStateTimer.reset();
                }
                break;
            case DOWN_SETTLE:
                // Give the ball a moment to roll onto the kicker while down
                if (kickerStateTimer.seconds() >= kickerSettleTime){
                    kickerState = KickerCycleState.WAIT_FOR_RPM;
                }
                break;
            case WAIT_FOR_RPM:
                // Only fire when flywheel is at speed based on RPM feedback
                if (getRPM() >= RPM - 500){
                    kickerGrav.up();
                    kickerState = KickerCycleState.MOVING_UP;
                    kickerStateTimer.reset();
                }
                break;
            case MOVING_UP:
                if (kickerGrav.isAtUpPosition()){
                    launched = true;
                    kickerGrav.down();
                    kickerState = KickerCycleState.MOVING_DOWN;
                    kickerStateTimer.reset();
                }
                break;
            case MOVING_DOWN:
                if (kickerGrav.isAtDownPosition()){
                    if (launched){
                        kickerCycleCount++;
                        launched = false;
                    }
                    kickerState = KickerCycleState.DOWN_SETTLE;
                    kickerStateTimer.reset();
                }
                break;
            default:
                break;
        }
    }

    public int getKickerCycleCount(){
        return kickerCycleCount;
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
    }

    public double getCurrentCycleTime(){
        return interval.seconds();
    }

    public void shortAuto(){
        double RPM = 2700;


        setRPM(RPM);
        enableKickerCycle(true,RPM);


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