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
        public static double farRPM = 3300;
        public static double closeRPM = 2600;
        public static double[] pidf = {268, 14.99, 0, 14.99};
        public static double gearRatio = 18.0/16.0;

        public static double kickerWaitTIme = 2;
        public static double kickerSettleTime = 0.2;
    }

    public ColorSensor hardwareColorSensor = null;
    public DcMotorEx outtakeMotor = null;
    private KickerSpindex kicker = null;
    private boolean isActive = false;
    public boolean launched = false;

    public Limelight limelight = null;
    private int encoderCount = 0;
    private boolean isFarLocation = true; // true = far (77%), false = short (55%)
    private boolean isBoosted = false; // Track if we're currently boosting
    private ElapsedTime boostTimer = new ElapsedTime(); // Timer for boost duration
    private int updateCounter = 0; // Counter for RPM checking interval
    private double lastRPM = 0; // Store last RPM reading
    private int prevEncoderPos = 0;
    private ElapsedTime rpmCalcTimer = new ElapsedTime();
    private double calculatedRPM = 0;
    private int kickerCycleCount = 0;
    private enum KickerCycleState { IDLE, ENSURE_DOWN, DOWN_SETTLE, WAIT_FOR_RPM, MOVING_UP, MOVING_DOWN }
    private KickerCycleState kickerState = KickerCycleState.IDLE;
    private ElapsedTime kickerStateTimer = new ElapsedTime();

    Turret turret;

    //The "E"ncoder "R"esolution our current motor runs at.
    int motorER = 28;

    private ElapsedTime clock = new ElapsedTime();
    //Interval in seconds of outtake cycle
    private ElapsedTime interval = new ElapsedTime();
    // Constructor - initializes the intake motor


    public Outtake(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
       outtakeMotor.setVelocityPIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public Outtake(HardwareMap hardwareMap, KickerSpindex kicker){
        this(hardwareMap);
        if (kicker != null){
            this.kicker = kicker;
        }
        else{
            throw new RuntimeException("The kicker object passed was uninitialized! (NULL)");
        }
    }

    public Outtake(HardwareMap hardwareMap, KickerSpindex kicker, Turret turret){
        this(hardwareMap);
        if (kicker != null){
            this.kicker = kicker;
            this.turret = turret;
        }
        else{
            throw new RuntimeException("The kicker object passed was uninitialized! (NULL)");
        }
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

    public void setPower(double x){
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor.setPower(x);}

    // Get current location name
    public String getLocationName() {
        return isFarLocation ? "FAR" : "SHORT";
    }

    /**
    @param distance hotizontally from goal
            **/
    public double distanceToRPM(double distance){

        double x =distance;

        double RPS =  (Math.sqrt( (9.8*x*x)/((0.075)*((5.062*x)-0.454025))))/0.603;

        return RPS * 60;

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
        double velocity = outtakeMotor.getVelocity();
        if (Math.abs(velocity) > 1) {
            return ((velocity * 60) / 28 / gearRatio);
        }
        int currentPos = outtakeMotor.getCurrentPosition();
        double dt = rpmCalcTimer.seconds();
        if (dt >= 0.02) {
            double ticksPerSec = (currentPos - prevEncoderPos) / dt;
            calculatedRPM = Math.abs((ticksPerSec * 60.0) / 28.0 / gearRatio);
            prevEncoderPos = currentPos;
            rpmCalcTimer.reset();
        }
        return calculatedRPM;
    }


    public void enableKickerCycle(boolean x, double RPM){
        if (kicker == null){
            throw new RuntimeException("You must pass the kicker object to the Outtake constructor!");
        }
        double time = interval.seconds();
        if (x){
            if (time >= 1 && time < 2 && getRPM() >= RPM-500){
                kicker.up();
                launched = true;
            }
            else if (time >= 2){
                kicker.down();
                if (launched){
                    kickerCycleCount++;
                }
                launched = false;
                interval.reset();
            }
        }
        else{
            kicker.up();
            interval.reset();
        }
    }


    public void enableSpindexKickerCycle(boolean x, double RPM){
        if (kicker == null){
            throw new RuntimeException("You must pass the kicker object to the Outtake constructor!");
        }
        double time = interval.seconds();
        //System.out.printf("enableSpindexKickerCycle: %.3f,%.1f,%.1f%n",time,getRPM(),RPM);
        if (x){
            // Phase 1: Wait for flywheel RPM, then kick up and start the timer

            if (!launched && Math.abs(RPM-getRPM()) <=100){
                kicker.up();
                launched = true;
                interval.reset(); // Timer starts when kick actually begins
                //System.out.printf("enableSpindexKickerCycle: Kicker up%n");
            }
            // Phase 2: After a full 200ms of kick travel, bring it back down
            else if (launched && time >= .1){
                kicker.down();
                kickerCycleCount++;
                launched = false;
                interval.reset();
                //System.out.printf("enableSpindexKickerCycle: Kicker down%n");
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

    public String getKickerState(){
        return kickerState.name();
    }

    public void resetKickerCycle(){
        if (kicker == null){
            throw new RuntimeException("You must pass the kicker object to the Outtake constructor!");
        }
        kickerCycleCount = 0;
        launched = false;
        interval.reset();
        kickerStateTimer.reset();
        kickerState = KickerCycleState.IDLE;
        //System.out.printf("Outtake.getKickerState:resetKickerCycle%n");
        kicker.down();
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