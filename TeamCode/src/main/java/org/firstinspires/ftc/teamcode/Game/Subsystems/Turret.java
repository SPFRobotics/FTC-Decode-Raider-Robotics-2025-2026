package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret {

    public CRServo rotation = null;

    public Outtake outtake = null;

    public Servo hood = null;


    public AnalogInput rotateEnconder = null;
    private Limelight limelight = null;

    // Piecewise P-control
    private static final double thresholdDeg = 30.0; // full power when outside this error
    private static final double toleranceDeg = 1.0;  // stop when inside this error
    private static final double maxPower = 0.5;      // cap CRServo power
    private static final double kP = maxPower / thresholdDeg; // proportional slope inside threshold
    private static final double searchP = 0.12;      // slow scan speed when tag not seen

    public Turret(HardwareMap hardwareMap){

        rotation = hardwareMap.get(CRServo.class, "turretServo");
        hood = hardwareMap.get(Servo.class,"hoodServo");
        rotateEnconder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        limelight = new Limelight(hardwareMap);
        outtake = new Outtake(hardwareMap);
        limelight.start();

    }


    public void realign(boolean team){

        // team == true  -> Red alliance goal (tag 24)
        // team == false -> Blue alliance goal (tag 20)
        final int targetTagId;
        if (team) {
            targetTagId = 24;
        } else {
            targetTagId = 20;
        }

        LLResult result = limelight.getLatestResult();

        // If we do not have a valid vision result, slowly scan to find the tag.
        if (result == null || !result.isValid()) {

            if (getPos() < 180 && getPos() > 0) {
                rotation.setPower(searchP);
            }else {
                rotation.setPower(-searchP);
            }

        }




        // Only react when the correct alliance tag is in view.
        int seenTag = limelight.getShootingAprilTagId();
        if (seenTag != targetTagId) {
            rotation.setPower(searchP);
            return;
        }

        // tx is horizontal offset in degrees: positive = target to the right, negative = left.
        double error = result.getTx();
        double sign = Math.signum(error);

        // Full power outside the threshold.
        if (Math.abs(error) > thresholdDeg) {
            rotation.setPower(maxPower * sign);
            return;
        }

        // Scaled power inside threshold but outside tolerance.
        if (Math.abs(error) > toleranceDeg) {
            rotation.setPower(error * kP);
            return;
        }

        // Stop when inside tolerance.
        rotation.setPower(0);



    }

    public double getPos(){
        return (rotateEnconder.getVoltage()/3.3)*360.0;
    }
    public void moveToPos(int target){

        double currentPos = (rotateEnconder.getVoltage())/3.3*360;

        double error = AngleUnit.normalizeDegrees(target - currentPos);

        double sign = Math.signum(error);

        double Threshold = 30;

        double maxPower = 0.1;

        double tolorence = 5;

        double kp = maxPower/Threshold;

        if(Math.abs(error) > Threshold){

            rotation.setPower(maxPower * sign);

        } else if (Math.abs(error) > tolorence) {

            rotation.setPower(error * kp);

        }else {
            rotation.setPower(0);
        }


    }



    public void autoHoodPos(){

        double distance = limelight.getDistanceFromTag();
        double RPM = outtake.getRPM();



    }

    public void setHoodPos(double x){hood.setPosition(x);}



}
