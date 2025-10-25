package org.firstinspires.ftc.teamcode.Game;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    @Config
    public static class Speed{
        public static double farPower = 0.77;     // Far location power (77%)
        public static double shortPower = 0.55;   // Short location power (55%)
        public static double power = 0;
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
    
    // Fixed boost parameters (no longer configurable)
    private static final double BOOST_POWER = 1.0;        // 100% boost power
    private static final double BOOST_DURATION = 0.5;     // 0.5 seconds boost duration
    private static final double RPM_THRESHOLD = 60;       // RPM threshold to trigger boost
    private static final int CHECK_INTERVAL = 5;          // Check RPM every 5 updates




    private ElapsedTime clock = new ElapsedTime();
    //Interval in seconds of outtake cycle
    private ElapsedTime interval = new ElapsedTime();
    // Constructor - initializes the intake motor
    public Outtake(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        kicker = new Kicker(hardwareMap);
    }


    // Update method - call this in the main loop with the gamepad
    public void update() {
        // Set motor power based on active state and location
        if (isActive) {
            // Check if boost timer has expired
            if (isBoosted && boostTimer.seconds() >= BOOST_DURATION) {
                isBoosted = false; // End boost phase
            }
            
            // Auto-boost detection: Check RPM periodically
            updateCounter++;
            if (updateCounter >= CHECK_INTERVAL) {
                updateCounter = 0;
                double currentRPM = getRPM(28);
                
                // If RPM drops below threshold and we're not already boosting, trigger boost
                if (currentRPM < RPM_THRESHOLD && !isBoosted && currentRPM > 0) {
                    isBoosted = true;
                    boostTimer.reset();
                }
                
                lastRPM = currentRPM;
            }
            
            // Determine power: boost power when boosted, location power otherwise
            double power;
            if (isBoosted) {
                power = BOOST_POWER;
            } else {
                power = isFarLocation ? Speed.farPower : Speed.shortPower;
            }
            
            outtakeMotor.setPower(power);
        } else {
            outtakeMotor.setPower(0.0); // Off when inactive
            isBoosted = false; // Reset boost when inactive
            updateCounter = 0; // Reset counter
            lastRPM = 0; // Reset RPM tracking
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

    public void setPower(double power) {
        outtakeMotor.setPower(power);
    }

    public void automate(boolean x){
        if (x){
            if (interval.seconds() >= 4){
                interval.reset();
            }
            if ((int)interval.seconds() == 0){
                kicker.down();
            }
            else if ((int)interval.seconds() == 2) {
                kicker.up();
            }
        }
        else{
            kicker.up();
        }
    }

    public double getCurrentCycleTime(){
        return interval.seconds();
    }

    public double getRPM(double encoderRes){
        int currentPos = outtakeMotor.getCurrentPosition();
        int deltaTicks = currentPos - encoderCount;

        if (deltaTicks != 0) {
            encoderCount = currentPos;
            double time = clock.seconds();
            clock.reset();

            double rotationsMoved = (double) deltaTicks / encoderRes;
            return (rotationsMoved / time)*60;
        }
        else{
            return 0;
        }
    }
    
    // Trigger boost for shooting (call this when you want to shoot)
    public void triggerBoost() {
        if (isActive) {
            isBoosted = true;
            boostTimer.reset(); // Start the boost timer
        }
    }
    
    // Get current power level (useful for telemetry)
    public double getCurrentPower() {
        if (isActive) {
            if (isBoosted) {
                return BOOST_POWER;
            } else {
                return isFarLocation ? Speed.farPower : Speed.shortPower;
            }
        }
        return 0.0;
    }
    
    // Check if currently boosting
    public boolean isBoosted() {
        return isBoosted;
    }
    
    // Get remaining boost time
    public double getRemainingBoostTime() {
        if (isBoosted) {
            return Math.max(0, BOOST_DURATION - boostTimer.seconds());
        }
        return 0.0;
    }
}