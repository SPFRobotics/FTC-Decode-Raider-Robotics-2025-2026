package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Spindex {
    @Config
    public static class SpindexValues{
        public static double range = 5.0; // Tolerance in degrees for position locking
        public static double kP = 0.01;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kStatic = 0.0; // Minimum effort to overcome servo stiction
        public static double integralZone = 20.0; // Only integrate inside this error band
        public static double maxIntegral = 200.0;
        public static double maxPower = 1.0;
    }
    private CRServo spindex = null;
    private static AnalogInput spindexPos = null;

    private double[] intakePos = {0, 240, 120};
    private double[] outtakePos = {60, 300, 180};

    private boolean mode = false;
    private int index = 0;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double targetPositionDeg = Double.NaN;
    private boolean controllerInitialized = false;
    private double lastCommandedPower = 0;

    public Spindex(HardwareMap hardwareMap){
        spindex = hardwareMap.get(CRServo.class, "spindex");
        spindexPos = hardwareMap.get(AnalogInput.class, "encoder");
        pidTimer.reset();
        refreshTarget();
    }

    public void addIndex(){
        index = (index + 1) % 3;
        refreshTarget();
    }

    public void subtractIndex(){
        index = (index - 1 + 3) % 3;
        refreshTarget();
    }
    //Locks on position based on the index
    public boolean getLockPos(){
        return mode;
    }
    public void lockPos(boolean mode){
        this.mode = mode;
        refreshTarget();

        double currentPos = getPos();
        double error = shortestDistance(targetPositionDeg, currentPos);
        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (!controllerInitialized) {
            dt = 0;
        }

        if (Math.abs(error) < SpindexValues.range){
            applyPower(0);
            lastError = error;
            integralSum = 0;
            controllerInitialized = false;
        }
        else {
            if (dt > 0 && Math.abs(error) < SpindexValues.integralZone){
                integralSum += error * dt;
                integralSum = Range.clip(integralSum, -SpindexValues.maxIntegral, SpindexValues.maxIntegral);
            } else if (Math.abs(error) >= SpindexValues.integralZone) {
                integralSum = 0;
            }

            double derivative = (dt > 0 && controllerInitialized) ? (error - lastError) / dt : 0;
            lastError = error;
            controllerInitialized = true;

            double power = SpindexValues.kP * error
                    + SpindexValues.kI * integralSum
                    + SpindexValues.kD * derivative;

            if (SpindexValues.kStatic != 0 && power != 0){
                power += Math.copySign(SpindexValues.kStatic, power);
            }

            power = Range.clip(power, -SpindexValues.maxPower, SpindexValues.maxPower);
            applyPower(power);
        }
    }

    public void zero(){
        resetControllerState();
        if (Spindex.getPos() <= 20){
            applyPower(0);
        }
        else{
            applyPower(0.1);
        }
    }

    public void move(double x){
        resetControllerState();
        applyPower(x);
    }

    public static double getPos(){
        return spindexPos.getVoltage() / 3.3 * 360;
    }

    public double getTargetPosition(){
        return targetPositionDeg;
    }

    public double getDistanceToTarget(){
        return lastError;
    }

    public double getLastCommandedPower(){
        return lastCommandedPower;
    }

    private void refreshTarget(){
        double newTarget = mode ? outtakePos[index % outtakePos.length] : intakePos[index % intakePos.length];
        if (Double.isNaN(targetPositionDeg) || Math.abs(shortestDistance(newTarget, targetPositionDeg)) > 1e-6){
            targetPositionDeg = newTarget;
            resetControllerState();
        } else {
            targetPositionDeg = newTarget;
        }
    }

    private double shortestDistance(double target, double current){
        double error = target - current;
        while (error > 180){
            error -= 360;
        }
        while (error < -180){
            error += 360;
        }
        return error;
    }

    private void resetControllerState(){
        integralSum = 0;
        lastError = 0;
        controllerInitialized = false;
        pidTimer.reset();
    }

    private void applyPower(double power){
        lastCommandedPower = Range.clip(power, -1.0, 1.0);
        spindex.setPower(lastCommandedPower);
    }
}
