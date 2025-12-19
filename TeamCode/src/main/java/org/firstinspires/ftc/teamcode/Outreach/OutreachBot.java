package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="OutreachBot")
public class OutreachBot extends LinearOpMode {
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;

    public void runOpMode() {
        // === HARDWARE MAPPING (unchanged) ===
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        // === ZERO POWER BEHAVIOR (unchanged) ===
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === MOTOR DIRECTIONS (unchanged) ===
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // === MODE VARIABLES ===
        boolean cosmeticMode = false;
        boolean triggersDown = false;

        waitForStart();

        while (opModeIsActive()) {

            // === FIXED TRIGGER LOGIC (same behavior, actually works now) ===
            boolean bothTriggers = (gamepad1.right_trigger >= 0.9 && gamepad1.left_trigger >= 0.9);

            if (bothTriggers && !triggersDown) {
                cosmeticMode = !cosmeticMode; // toggle mode
                triggersDown = true;
            }

            if (!bothTriggers) {
                triggersDown = false;
            }

            // === COSMETIC SPIN MODE (unchanged movement) ===
            if (cosmeticMode) {
                rightFront.setPower(0.25);
                leftFront.setPower(-0.25);
                rightBack.setPower(0.25);
                leftBack.setPower(-0.25);
            }

            // === NORMAL DRIVE MODE (fixed math, same control scheme) ===
            else {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denominator = Math.max(1, Math.abs(y) + Math.abs(x) + Math.abs(rx));

                leftFront.setPower((y + x + rx) / denominator);
                leftBack.setPower((y - x + rx) / denominator);
                rightFront.setPower((y - x - rx) / denominator);
                rightBack.setPower((y + x - rx) / denominator);
            }
        }
    }
}

