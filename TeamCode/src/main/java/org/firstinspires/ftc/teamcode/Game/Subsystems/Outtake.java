package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    @Config
    public static class OuttakeSpeed{
        public static double farRPM = 3200;
        public static double closeRPM = 2700;
        public static double reverseRPM = -200;
        public static double p = 1.5311682242990654205607476635514018691588785046728971962616822429906542056074766355140186915887850467;
        public static double i = 0.1531168224299065420560747663551401869158878504672897196261682242990654205607476635514018691588785047;
        public static double d = 0;
        public static double f = 15.3116822429906542056074766355140186915887850467289719626168224299065420560747663551401869158878504673;
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
        outtakeMotor.setVelocityPIDFCoefficients(OuttakeSpeed.p, OuttakeSpeed.i, OuttakeSpeed.d, OuttakeSpeed.f);
        outtakeMotor.setPositionPIDFCoefficients(5);
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
        double tps = ((rpm/60)*motorER)/1.0625;
        outtakeMotor.setVelocity(tps);
    }
}
