package org.firstinspires.ftc.teamcode.Resources;

public class Unit {
    public static double inch_convert(double inch) { return inch * (537.7 / (3.78 * Math.PI)); } // converts inches to ticks
    public static double inchToLift_convert(double inch) { return inch * (4062 / 51.5); }
    public static double inToCm(int inches) { return inches * 2.54; } // converts inches to cm
    public static double cm_convert(double cm) { return cm * (537.7 / (9.6012 / Math.PI)); } // converts cm to ticks
}
