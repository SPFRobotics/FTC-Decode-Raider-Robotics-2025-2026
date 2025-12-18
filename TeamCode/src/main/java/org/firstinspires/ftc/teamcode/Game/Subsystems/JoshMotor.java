package org.firstinspires.ftc.teamcode.Game.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class JoshMotor {

    private DcMotor joshMotor;

    public JoshMotor(HardwareMap hardwareMap) {
        joshMotor = hardwareMap.get(DcMotor.class, "JoshMotor");
        joshMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(Gamepad gamepad1) {
        if (gamepad1.right_trigger > 0.05) {
            joshMotor.setPower(0.5);
        }
        else if (gamepad1.left_trigger > 0.05) {
            joshMotor.setPower(-0.5);
        }
        else {
            joshMotor.setPower(0);
        }
    }
}
