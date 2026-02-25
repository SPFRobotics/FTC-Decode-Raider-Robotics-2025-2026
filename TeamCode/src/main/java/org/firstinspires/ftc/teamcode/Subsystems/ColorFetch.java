package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.ColorFetch.ColorFetchConfig.loops;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Resources.Unit;

import java.util.Arrays;

public class ColorFetch {
    @Config
    public static class ColorFetchConfig{
        public static int loops = 5;
    }

    /*****************Class varibles**********************/
    private NormalizedColorSensor colorSensor = null;
    private RevColorSensorV3 distanceSensor = null;
    /*****************************************************/

    /**************************************Constructor******************************************/
    public ColorFetch(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
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

    double combined = 0;
    public double getAverageHue(){
        if (combined > 0){
            combined = 0;
        }
        for (int i = 0; i < loops; i++){
            combined += getHue();
        }
        return combined/loops;
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

    public char getAverageColor(){
        double hue = getAverageHue();
        if (hue >= 200.0 && hue <= 240.0){
            return 'P';
        }
        else if (hue >= 100.0 && hue <= 170.0){
            return 'G';
        }
        else{
            return 'E';
        }
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }


    public void showTelemetry(Telemetry telemetry){
        telemetry.addLine("------------------------------------------------------------------------------------");
        telemetry.addLine("Color Fetch");
        telemetry.addLine("HSV: " + Arrays.toString(getHSVArray()));
        telemetry.addLine("Color: " + getColor());
        telemetry.addLine("Average Hue: " + getAverageHue());
        telemetry.addLine("Average Color: " + getAverageColor());
        telemetry.addLine("Distance: " + getDistance());
        telemetry.addLine("------------------------------------------------------------------------------------");
    }

    public void showTelemetry(MultipleTelemetry telemetry){
        telemetry.addLine("------------------------------------------------------------------------------------");
        telemetry.addLine("Color Fetch");
        telemetry.addLine("HSV: " + Arrays.toString(getHSVArray()));
        telemetry.addLine("Color: " + getColor());
        telemetry.addLine("Average Hue: " + getAverageHue());
        telemetry.addLine("Average Color: " + getAverageColor());
        telemetry.addLine("Distance: " + getDistance());
        telemetry.addLine("------------------------------------------------------------------------------------");
    }
}
