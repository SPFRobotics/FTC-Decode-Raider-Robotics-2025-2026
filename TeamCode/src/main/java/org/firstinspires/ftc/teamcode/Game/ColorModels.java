package org.firstinspires.ftc.teamcode.Game;

public class ColorModels {
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
