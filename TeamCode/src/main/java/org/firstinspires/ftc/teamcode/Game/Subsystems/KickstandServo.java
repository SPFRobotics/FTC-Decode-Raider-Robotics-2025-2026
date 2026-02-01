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
        public static int up = 20;
        public static int down = 0;
        public static double threshold = 15;
        public static double tolorance = 2;
        public static boolean reverseDir = true;
    }
    /*########################################*/

    /*##############CLASS VARIABLES##############*/
    CRServo kickstand = null;
    AnalogInput kickstandPos = null;
    int relPos = 0;
    int prevPos = 0;
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
         if ((int)(getPosition() - prevPos) >= 2){
             relPos++;
         }
         else if ((int)(getPosition() - prevPos) <= -2){
             relPos--;
         }

         int error = (int)target - (int)relPos;
         int sign = (int)Math.signum(error);
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

         prevPos = (int)getPosition();

    }
    /*#################################################*/
}
