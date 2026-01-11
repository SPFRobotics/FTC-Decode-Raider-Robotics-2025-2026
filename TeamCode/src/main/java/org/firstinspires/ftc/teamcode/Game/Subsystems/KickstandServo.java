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
        public static double threshold = 3;
        public static boolean reverseDir = true;
    }
    /*########################################*/

    /*##############CLASS VARIABLES##############*/
    CRServo kickstand = null;
    AnalogInput kickstandPos = null;
    double currentPos = 0;
    double difference = 0;
    /*###########################################*/

    public KickstandServo(HardwareMap hardwareMap){
        kickstand = hardwareMap.get(CRServo.class, "kickstand");
        kickstandPos = hardwareMap.get(AnalogInput.class, "kickstandPos");
        if (reverseDir){
            kickstand.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        currentPos = getPosition();
    }

    /*#####################Methods#####################*/

    public void setPower(double x){
        kickstand.setPower(x);
    }

    public double getPosition(){
        return (kickstandPos.getVoltage()/3.3*360.0)*(2.0/15.0);
    }

    public double getDifference(){
        return difference;
    }

    //Up as in being in the "idle" position
    public void up(){
        currentPos = getPosition();
        difference = Math.abs(up-currentPos);

        if (difference > threshold){
            setPower(power);
        }
        else{
            setPower(0);
        }
    }

    public void down(){
        currentPos = getPosition();
        difference = Math.abs(down-currentPos);

        if (difference > threshold){
            setPower(-power);
        }
        else{
            setPower(0);
        }
    }
    /*#################################################*/
}
