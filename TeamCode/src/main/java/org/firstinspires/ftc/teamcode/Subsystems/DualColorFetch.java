package org.firstinspires.ftc.teamcode.Subsystems;
import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DualColorFetch extends ColorFetch{
    private NormalizedColorSensor colorSensor = null;
    private RevColorSensorV3 distanceSensor = null;
    private NormalizedColorSensor colorSensor2 = null;
    private RevColorSensorV3 distanceSensor2 = null;

    public DualColorFetch(HardwareMap hardwareMap){
        super(hardwareMap);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        distanceSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
    }

    public double[] getDistances(){
        return new double[]{distanceSensor.getDistance(DistanceUnit.CM), distanceSensor2.getDistance(DistanceUnit.CM)};
    }

    public float[][] getHSVArrays(){
        int[] colorInteger = {colorSensor.getNormalizedColors().toColor(), colorSensor2.getNormalizedColors().toColor()};
        float[][] hsv = new float[3][2];
        for (int i = 0; i < 2; i++){
            Color.colorToHSV(colorInteger[i], hsv[i]);
        }
        return hsv;
    }
}
