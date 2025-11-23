package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensor {

    private com.qualcomm.robotcore.hardware.ColorSensor hardwareColorSensor = null;
    
    // Constructor with HardwareMap - initializes the color sensor
    public ColorSensor(HardwareMap hardwareMap) {
        hardwareColorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor");
    }
    
    // Returns the color data as RGB array [r, g, b]
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
        
        // Purple: Hue range 200-220 (from test program)
        return hue >= 200 && hue <= 220;
    }
    
    public boolean isGreen() {
        if (hardwareColorSensor == null) {
            return false;
        }
        int[] rgb = getColor();
        int[] hsv = rgbToHSV(rgb[0], rgb[1], rgb[2]);
        int hue = hsv[0];
        
        // Green: Hue range 155-160 (from test program)
        return hue >= 155 && hue <= 160;
    }
    
    public boolean isPurpleOrGreen() {
        return isPurple() || isGreen();
    }
    
    public int[] rgbToHSV(int r, int g, int b){
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
