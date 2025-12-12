package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    public CRServo rotation = null;
    private Limelight limelight = null;

    private static final double kP = 0.015;      // proportional gain for tx error -> servo power
    private static final double tolerance = 1.0; // degrees of tx considered "centered"
    private static final double maxP = 0.5;       // cap CRServo power to avoid overshoot
    private static final double searchP = 0.12;   // slow scan speed when tag not seen

    public Turret(HardwareMap hardwareMap){

        rotation = hardwareMap.get(CRServo.class, "TurretServo");
        limelight = new Limelight(hardwareMap);
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
            rotation.setPower(searchP);
            return;
        }

        // Only react when the correct alliance tag is in view.
        int seenTag = limelight.getShootingAprilTagId();
        if (seenTag != targetTagId) {
            rotation.setPower(searchP);
            return;
        }

        // tx is horizontal offset in degrees: positive = target to the right, negative = left.
        double tx = result.getTx();

        // Simple P-control to drive the offset toward zero.
        double power = tx * kP;

        // Clamp power to keep motion smooth and safe.
        if (power > maxP) power = maxP;
        if (power < -maxP) power = -maxP;

        // Stop when inside tolerance.
        if (Math.abs(tx) < tolerance) {
            rotation.setPower(0);
            return;
        }

        rotation.setPower(power);



    }


}
