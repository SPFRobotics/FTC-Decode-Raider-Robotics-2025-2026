package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//ROYGBIV!!!
public class LedLights {
    private Servo ledServo;
//Constructor
    public LedLights(String led, HardwareMap hardwareMap) {
        hardwareMap.get(Servo.class, led);
    }
//Set red
    public void setRed() {
        ledServo.setPosition(0.277);
    }
//Set orange
    public void setOrange() {
        ledServo.setPosition(0.333);
    }
//set Yellow
    public void setYellow() {
        ledServo.setPosition(0.338);
    }
//Set weird green
    public void setSage(){
        ledServo.setPosition(0.444);
    }
//set green
    public void setGreen() {
        ledServo.setPosition(0.5);
    }
    //set weird blue
    public void setAzure() {
        ledServo.setPosition(0.555);
    }
    //set blue
    public void setBlue() {
        ledServo.setPosition(0.611);
    }
//set indigo
    public void setIndigo() {
        ledServo.setPosition(0.666);
    }
//set violet
    public void setViolet() {
        ledServo.setPosition(0.722);
    }
//set white
    public void setWhite(){

        ledServo.setPosition(1);
    }
//turn off
    public void turnOFF(){
        ledServo.setPosition(0);
    }


}
