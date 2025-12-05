package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

public class Outtake {
    private Outtake outtake;

    @Config
    public static class OuttakeSpeed{
        public static double farRPM = 3200;
        public static double closeRPM = 2700;
        public static double sortRPM = 1000;
        public static double p = 100;
        public static double i = 0;
        public static double d = 0;
        public static double f = 0;
    }

    private ColorFinder colorFinder = null;
    public ColorSensor hardwareColorSensor = null;
    public DcMotorEx outtakeMotor = null;
    private Kicker kicker = null;
    private boolean isActive = false;

    public Limelight limelight = null; 
    private int encoderCount = 0;
    private boolean isFarLocation = true; // true = far (77%), false = short (55%)
    private boolean isBoosted = false; // Track if we're currently boosting
    private ElapsedTime boostTimer = new ElapsedTime(); // Timer for boost duration
    private int updateCounter = 0; // Counter for RPM checking interval
    private double lastRPM = 0; // Store last RPM reading
    private int kickerCycleCount = 0;

    //The "E"ncoder "R"esolution our current motor runs at.
    int motorER = 28;

    private ElapsedTime clock = new ElapsedTime();
    //Interval in seconds of outtake cycle
    private ElapsedTime interval = new ElapsedTime();
    // Constructor - initializes the intake motor
    public Outtake(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setVelocityPIDFCoefficients(OuttakeSpeed.p, OuttakeSpeed.i, OuttakeSpeed.d, OuttakeSpeed.f);
        //outtakeMotor.setPositionPIDFCoefficients(5);
        kicker = new Kicker(hardwareMap);
        limelight = new Limelight(hardwareMap);
    }


    
    public void ColorSort(){
        
        limelight.getMotifId();
        LLResult result = limelight.getLatestResult();
        outtake.setRPM(Outtake.OuttakeSpeed.sortRPM);


        if(result.equals(22)&& colorFinder.isGreen()&&kickerCycleCount==1||kickerCycleCount==3){
            outtake.enableKickerCycle(true, OuttakeSpeed.sortRPM);
            kicker.down(true);

        }
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
        return ((outtakeMotor.getVelocity()*60)/28);
    }


    public void enableKickerCycle(boolean x, double RPM){
        if (x){
            if ((int)interval.seconds() >= 2 && getRPM() >= RPM) {
                kicker.up(true);
            }
            else if ((int)interval.seconds() >= 5){
                kickerCycleCount++;
                interval.reset();
            }
        }
        else{
            kicker.down(true);
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
        rpm *= 1.125;
        double tps = (rpm / 60.0) * 28;
        outtakeMotor.setVelocity(tps);
    }
}
