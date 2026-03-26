package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.KickstandServo.KickstandServoConfig.*;

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
    CRServo kickstand1 = null;
    CRServo kickstand2 = null;

    AnalogInput kickstandPos2 = null;
    AnalogInput kickstandPos1 = null;
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
        kickstand1 = hardwareMap.get(CRServo.class, "kickstand1");
        kickstandPos1 = hardwareMap.get(AnalogInput.class, "kickstandPos1");
        kickstand2 = hardwareMap.get(CRServo.class, "kickstand2");
        kickstandPos2 = hardwareMap.get(AnalogInput.class, "kickstandPos2");
        kickstand1.setPower(0);
        if (reverseDir){
            kickstand1.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    /*#####################Methods#####################*/

    public void setPower(double x){
        kickstand1.setPower(x);
    }

    public double getPosition(){
        return (kickstandPos1.getVoltage()/3.3*360.0);
    }

    public double getVoltage(){
        return kickstandPos1.getVoltage();
    }


    public void updatePos(int target){
         double error = target - getPosition();
         double sign = Math.signum(error);
         double kp = 1/threshold;

         if (Math.abs(error) > threshold){
             kickstand1.setPower(sign);
             kickstand2.setPower(sign);
         }
         else if (Math.abs(error) > tolorance){
             kickstand1.setPower(error * kp);
             kickstand2.setPower((error*kp));
         }
         else {
             kickstand1.setPower(0);
             kickstand2.setPower(0);

         }
    }





    /*#################################################*/
}
