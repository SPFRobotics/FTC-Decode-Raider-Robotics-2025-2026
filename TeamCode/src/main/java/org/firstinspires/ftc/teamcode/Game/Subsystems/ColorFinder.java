package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorFinder {

    public ColorSensor hardwareColorSensor = null;
    public DistanceSensor distanceSensor = null;
    private static final int MIN_SAT = 10; // percent
    private static final int MIN_VAL = 5;  // percent
    
    // Constructor with HardwareMap - initializes the color sensor
    public ColorFinder(HardwareMap hardwareMap) {
        hardwareColorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
    }
    
    public int[] getColor() {
        int r = Math.min(hardwareColorSensor.red(), 255);
        int g = Math.min(hardwareColorSensor.green(), 255);
        int b = Math.min(hardwareColorSensor.blue(), 255);
        return new int[]{r, g, b};
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
    
    public boolean isPurple() {
        if (hardwareColorSensor == null) {
            return false;
        }
        int[] rgb = getColor();
        int[] hsv = rgbToHSV(rgb[0], rgb[1], rgb[2]);
        int hue = hsv[0];
        if (hsv[1] < MIN_SAT || hsv[2] < MIN_VAL) return false;
        
        // Purple range: main band 240-320 plus wrap 0-30
        return (hue >= 240 && hue <= 320) || (hue >= 0 && hue <= 30);
    }
    
    public boolean isGreen() {
        if (hardwareColorSensor == null) {
            return false;
        }
        int[] rgb = getColor();
        int[] hsv = rgbToHSV(rgb[0], rgb[1], rgb[2]);
        int hue = hsv[0];
        if (hsv[1] < MIN_SAT || hsv[2] < MIN_VAL) return false;
        
        // Green range: 90-170 covers typical FTC game pieces
        return hue >= 90 && hue <= 170;
    }
    
    public boolean isBlue() {
        if (hardwareColorSensor == null) {
            return false;
        }
        int[] rgb = getColor();
        int[] hsv = rgbToHSV(rgb[0], rgb[1], rgb[2]);
        int hue = hsv[0];
        
        return hue >= 240 && hue <= 260;
    }
    
    public boolean isPurpleOrGreen() {
        return isPurple() || isGreen();
    }
    
    public int[] rgbToHSV(int r, int g, int b) {
        double rPrime = r/255.0;
        double gPrime = g/255.0;
        double bPrime = b/255.0;

        double cMax = Double.max(rPrime, Double.max(gPrime, bPrime));
        double cMin = Double.min(rPrime, Double.min(gPrime, bPrime));

        double tri = cMax - cMin;

        int h = 0;
        int s = 0;
        int v = 0;

        if (tri == 0){
            h = 0;
        }
        else if (cMax == rPrime){
            h=(int)(60*(((gPrime-bPrime)/tri) % 6));
        }
        else if (cMax == gPrime){
            h=(int)(60*(((bPrime-rPrime)/tri)+2));
        }
        else if (cMax == bPrime){
            h=(int)(60*(((rPrime-gPrime)/tri)+4));
        }
        h = (h+360)%360;
        //Converts the RGB color model into HSV. HUE, SATURATION, VALUE: https://www.rapidtables.com/convert/color/rgb-to-hsv.html
        if (cMax != 0){
            s = (int)(tri/cMax*100);
        }

        v = (int)(cMax*100);

        return new int[]{h, s, v};
    }
}
