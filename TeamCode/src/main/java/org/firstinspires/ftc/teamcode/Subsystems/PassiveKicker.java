package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.PassiveKicker.PassiveKickerConfig.reverse;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PassiveKicker {
    Servo passiveKicker;
    @Config
    public static class PassiveKickerConfig{
        public static double upPos = 0;
        public static double downPos = 0;

        public static boolean reverse = false;

    }
    public PassiveKicker(HardwareMap hardwareMap){
        passiveKicker = hardwareMap.get(Servo.class, "passiveKicker");
        if (reverse) {
            passiveKicker.setDirection(Servo.Direction.REVERSE);
        } else {
            passiveKicker.setDirection(Servo.Direction.FORWARD);
        }

    }

    public void up(){
        passiveKicker.setPosition(PassiveKickerConfig.upPos);
    }

    public void down(){
        passiveKicker.setPosition(PassiveKickerConfig.downPos);
    }
}
