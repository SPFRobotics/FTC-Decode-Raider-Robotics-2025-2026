package org.firstinspires.ftc.teamcode.Game;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;



@Config
public class Scroll {
    private StringBuilder modTxt = new StringBuilder("");
    private int scrollCount = 0;
    private String txt = "";
    private ElapsedTime masterClock = new ElapsedTime();
    private double time = 1;


    public Scroll(String txt){
        this.txt = txt;
        modTxt.append(txt);
        modTxt.append("   ");
    }

    public String foward(){
        if (masterClock.seconds() >= time){
            scrollCount++;
            time = masterClock.seconds()+0.1;

            if (scrollCount == 0 || scrollCount == txt.length()){
                scrollCount = 0;
            }
            else{
                String letter = modTxt.substring(0, 1);
                modTxt.deleteCharAt(0);
                modTxt.append(letter);
            }
        }
        if (modTxt.length() > 48){
            return modTxt.toString().substring(0, 47);
        }
        else{
            return modTxt.toString();
        }
    }
}
