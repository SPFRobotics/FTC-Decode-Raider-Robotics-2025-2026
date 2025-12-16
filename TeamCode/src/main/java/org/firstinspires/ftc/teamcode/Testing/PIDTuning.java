package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;

@TeleOp(name="PIDTuning")
public class PIDTuning extends LinearOpMode {
    private DcMotorEx motor = null;
    double currentVelocity = 0;
    double maxVelocity = 0;

    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        motor.setPower(1);

        waitForStart();
        while (opModeIsActive()){
            currentVelocity = motor.getVelocity();
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
