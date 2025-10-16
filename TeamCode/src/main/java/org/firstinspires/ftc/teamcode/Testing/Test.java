package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Test extends LinearOpMode {
    public DcMotor Motor1 = null;
    public void runOpMode(){
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        waitForStart();
        while (opModeIsActive()){
            Motor1.setPower(gamepad1.right_trigger);
        }
    }
}
