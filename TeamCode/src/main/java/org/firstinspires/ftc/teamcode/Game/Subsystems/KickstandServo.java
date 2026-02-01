package org.firstinspires.ftc.teamcode.Game.Subsystems;

import static org.firstinspires.ftc.teamcode.Game.Subsystems.KickstandServo.KickstandServoConfig.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

public class KickstandServo {
    /*##############FTCDASHBOARD##############*/
    @Config
    public static class KickstandServoConfig{
        public static double power = 1;
        public static int up = 350;
        public static int down = 0;
        public static double threshold = 62;
        public static double tolorance = 5;
        public static boolean reverseDir = true;
    }
    /*########################################*/

    /*##############CLASS VARIABLES##############*/
    CRServo kickstand = null;
    AnalogInput kickstandPos = null;
    int relPos = 0;
    int prevPos = 0;
    
    // Variables for improved position tracking with filtering
    private double smoothedPosition = 0;
    private double prevSmoothedPosition = 0;
    private double accumulatedPosition = 0;  // Tracks total movement including wraparound
    private boolean isFirstReading = true;
    
    // Tunable parameters for the improved method
    private static final double SMOOTHING_FACTOR = 0.3;  // Lower = more smoothing (0.1-0.5 recommended)
    private static final double DEAD_ZONE_DEGREES = 5.0;  // Ignore movements smaller than this
    private static final double WRAPAROUND_THRESHOLD = 180.0;  // Detect wraparound if delta > this
    /*###########################################*/

    public KickstandServo(HardwareMap hardwareMap){
        kickstand = hardwareMap.get(CRServo.class, "kickstand");
        kickstandPos = hardwareMap.get(AnalogInput.class, "kickstandPos");
        kickstand.setPower(0);
        if (reverseDir){
            kickstand.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    /*#####################Methods#####################*/

    public void setPower(double x){
        kickstand.setPower(x);
    }

    public double getPosition(){
        return (kickstandPos.getVoltage()/3.3*360.0);
    }

    public double getVoltage(){
        return kickstandPos.getVoltage();
    }

    public int getRelPos(){
        return (int)relPos;
    }

    public void updatePos(int target){
         double error = target - getPosition();
         double sign = Math.signum(error);
         double kp = 1/threshold;

         if (Math.abs(error) > threshold){
             kickstand.setPower(sign);
         }
         else if (Math.abs(error) > tolorance){
             kickstand.setPower(error * kp);
         }
         else {
             kickstand.setPower(0);
         }
    }


    public void updatePosFiltered(int target) {
        double rawPosition = getPosition();  // 0-360 degrees
        
        // Initialize on first reading
        if (isFirstReading) {
            smoothedPosition = rawPosition;
            prevSmoothedPosition = rawPosition;
            accumulatedPosition = 0;
            isFirstReading = false;
            return;
        }
        
        smoothedPosition = SMOOTHING_FACTOR * rawPosition + (1 - SMOOTHING_FACTOR) * smoothedPosition;
        
        double delta = smoothedPosition - prevSmoothedPosition;
        
        if (delta > WRAPAROUND_THRESHOLD) {
            delta -= 360.0;
        } else if (delta < -WRAPAROUND_THRESHOLD) {
            delta += 360.0;
        }
        
        if (Math.abs(delta) > DEAD_ZONE_DEGREES) {
            accumulatedPosition += delta;
            prevSmoothedPosition = smoothedPosition;
        }
        
        double error = target - accumulatedPosition;
        int sign = (int) Math.signum(error);
        double kp = 1.0 / threshold;
        
        if (Math.abs(error) > threshold) {
            kickstand.setPower(sign * power);
        } else if (Math.abs(error) > tolorance) {
            kickstand.setPower(error * kp);
        } else {
            kickstand.setPower(0);
        }
    }
    

    public double getAccumulatedPosition() {
        return accumulatedPosition;
    }

    public void resetAccumulatedPosition() {
        accumulatedPosition = 0;
        isFirstReading = true;
    }
    /*#################################################*/
}
