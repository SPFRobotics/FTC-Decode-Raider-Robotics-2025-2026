package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Resources.Unit;

public class ColorFetch {
    /*****************Class varibles**********************/
    private NormalizedColorSensor colorSensor = null;
    private DistanceSensor distanceSensor = null;
    /*****************************************************/

    /**************************************Constructor******************************************/
    public ColorFetch(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        colorSensor.setGain(3.0f);
    }
    /******************************************************************************************/

    /**************************************Methods*********************************************/
    //Returns an array with the three HSV values (Hue, Saturation and Value)
    public float[] getHSVArray(){
        int colorInteger = colorSensor.getNormalizedColors().toColor();
        float[] hsv = new float[3];
        Color.colorToHSV(colorInteger, hsv);
        return hsv;
    }

    /*Returns only the hue from the array returned by the getHSVArray class*/
    public float getHue(){
        return getHSVArray()[0];
    }

    public float getSaturation(){
        return getHSVArray()[1];
    }

    /*Gets relevant colors. Returns P if purple G if green and E if empty*/
    public char getColor(){
        float hue = getHue();
        if (hue >= 200.0f && hue <= 240.0f){
            return 'P';
        }
        else if (hue >= 100.0f && hue <= 170.0f){
            return 'G';
        }
        else{
            return 'E';
        }
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
    /**************************************Methods*********************************************/
}
