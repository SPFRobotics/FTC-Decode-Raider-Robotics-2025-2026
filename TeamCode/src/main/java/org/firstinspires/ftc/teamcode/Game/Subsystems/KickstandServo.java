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
        public static double up = 0;
        public static double down = 0;
        public static boolean reverseDir = true;
    }
    /*########################################*/

    /*##############CLASS VARIABLES##############*/
    CRServo kickstand = null;
    AnalogInput kickstandPos = null;
    double relPos = 0;
    double prevPos = 0;
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

    public double updatePos(){
         if ((int)getPosition() > (int)prevPos){
             relPos++;
         }
         else if ((int)getPosition() < (int)prevPos){
             relPos--;
         }
         prevPos = getPosition();
         return relPos;
    }


    /*#################################################*/
}
