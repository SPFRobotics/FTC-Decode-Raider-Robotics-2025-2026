package org.firstinspires.ftc.teamcode.Game.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "JoshTele")
public class JoshMotor extends LinearOpMode {

    private DcMotor joshMotor;

    @Override
    public void runOpMode() {

        joshMotor = hardwareMap.get(DcMotor.class, "JoshMotor");
        joshMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

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
}
