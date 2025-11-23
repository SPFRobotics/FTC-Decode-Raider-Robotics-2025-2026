package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorFinder {

    private com.qualcomm.robotcore.hardware.ColorSensor hardwareColorSensor = null;
    
    // Constructor with HardwareMap - initializes the color sensor
    public ColorFinder(HardwareMap hardwareMap) {
        hardwareColorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor");
    }
    
    public int[] getColor() {
        if (hardwareColorSensor == null) {
            return new int[]{0, 0, 0};
        }
        int r = Math.min(hardwareColorSensor.red(), 255);
        int g = Math.min(hardwareColorSensor.green(), 255);
        int b = Math.min(hardwareColorSensor.blue(), 255);
        return new int[]{r, g, b};
    }
    
    public boolean isPurple() {
        if (hardwareColorSensor == null) {
            return false;
        }
        int[] rgb = getColor();
        int[] hsv = rgbToHSV(rgb[0], rgb[1], rgb[2]);
        int hue = hsv[0];
        
        // Purple: Wider range to account for variations (original was 200-220)
        // Purple can also appear near the wrap-around (270-360)
        return (hue >= 190 && hue <= 240) || (hue >= 270 && hue <= 360);
    }
    
    public boolean isGreen() {
        if (hardwareColorSensor == null) {
            return false;
        }
        int[] rgb = getColor();
        int[] hsv = rgbToHSV(rgb[0], rgb[1], rgb[2]);
        int hue = hsv[0];
        
        // Green: Wider range to account for variations
        return hue >= 120 && hue <= 180;
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
