package org.firstinspires.ftc.teamcode.Game.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

public class Outtake {
    @Config
    public static class OuttakeSpeed{
        public static double farRPM = 3200;
        public static double closeRPM = 2700;
        public static double reverseRPM = -200;
    }
    public DcMotorEx outtakeMotor = null;
    private Kicker kicker = null;
    private boolean isActive = false;
    private int encoderCount = 0;
    private boolean isFarLocation = true; // true = far (77%), false = short (55%)
    private boolean isBoosted = false; // Track if we're currently boosting
    private ElapsedTime boostTimer = new ElapsedTime(); // Timer for boost duration
    private int updateCounter = 0; // Counter for RPM checking interval
    private double lastRPM = 0; // Store last RPM reading
    private int kickerCycleCount = 0;

    //The "E"ncoder "R"esolution our current motor runs at.
    private final int motorER = 28;

    private ElapsedTime clock = new ElapsedTime();
    //Interval in seconds of outtake cycle
    private ElapsedTime interval = new ElapsedTime();
    // Constructor - initializes the intake motor
    public Outtake(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kicker = new Kicker(hardwareMap);
    }

    
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

    public double getRPM() {
        return (outtakeMotor.getVelocity()*60)/motorER;
    }

    public void enableKickerCycle(boolean x, double RPM){
        if (x){
            if ((int)interval.seconds() == 0){
                kicker.down();

            }
            else if ((int)interval.seconds() >= 2 && getRPM() >= RPM) {
                kicker.up();
            }
            else if ((int)interval.seconds() >= 5){
                kickerCycleCount++;
                interval.reset();
            }
        }
        else{
            kicker.up();
            interval.reset();
        }
    }

    public int getKickerCycleCount(){
        return kickerCycleCount;
    }

    public double getCurrentCycleTime(){
        return interval.seconds();
    }

    public void setRPM(double rpm){
        double tps = (rpm/60)*motorER;
        outtakeMotor.setVelocity(tps);

    }




}
