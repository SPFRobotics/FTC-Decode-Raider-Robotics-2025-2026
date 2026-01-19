package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
public class OdoPodTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left = hardwareMap.get(DcMotor.class, "backLeftDrive");
        DcMotor right = hardwareMap.get(DcMotor.class, "backRightDrive");
        DcMotor strafe = hardwareMap.get(DcMotor.class, "frontRightDrive");
        telemetry.setMsTransmissionInterval(16);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Left", left.getCurrentPosition());
            telemetry.addData("Right", right.getCurrentPosition());
            telemetry.addData("Strafe", strafe.getCurrentPosition());
            telemetry.update();
        }
    }
}
