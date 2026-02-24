package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Testing.SpindexTuning.SpindexTuningConfig.kD;
import static org.firstinspires.ftc.teamcode.Testing.SpindexTuning.SpindexTuningConfig.kI;
import static org.firstinspires.ftc.teamcode.Testing.SpindexTuning.SpindexTuningConfig.kP;
import static org.firstinspires.ftc.teamcode.Testing.SpindexTuning.SpindexTuningConfig.maxPower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


//@TeleOp(name = "Spindex PID Tuning", group = "Testing")
public class SpindexTuning extends OpMode {

    public static class SpindexTuningConfig {
        public static double kP = 20;
        public static double kI = 0.04;
        public static double kD = 0.0;
        public static double maxPower = 1;
    }


    private DcMotorEx spindexMotor;
    private static final double TICKS_PER_REV = 537.7;
    private int targetPosition = 0;

    @Override
    public void init() {
        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spindexMotor.setTargetPosition(0);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        telemetry.update();
    }

    @Override
    public void loop() {
        // RUN_TO_POSITION only supports PID (no F term) on the REV Hub
        spindexMotor.setPIDCoefficients(
                DcMotor.RunMode.RUN_TO_POSITION,
                new PIDCoefficients(kP, kI, kD)
        );

        if (gamepad1.right_bumper) {
            targetPosition = 500;
        } else if (gamepad1.left_bumper) {
            targetPosition = 0;
        }

        spindexMotor.setTargetPosition(targetPosition);
        spindexMotor.setPower(maxPower);

        int currentPos = spindexMotor.getCurrentPosition();
        int error = targetPosition - currentPos;
        double currentDeg = (currentPos / TICKS_PER_REV) * 360.0;
        double motorPower = spindexMotor.getPower();
        PIDCoefficients activePID = spindexMotor.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Position (ticks)", currentPos);
        telemetry.addData("Position (deg)", "%.1f", currentDeg);
        telemetry.addData("Error (ticks)", error);
        telemetry.addData("Motor Power", "%.3f", motorPower);
        telemetry.addData("Target (ticks)", targetPosition);
        telemetry.addData("At Target?", !spindexMotor.isBusy());
        telemetry.addLine();
        telemetry.addData("PID", "P=%.2f  I=%.4f  D=%.4f",
                activePID.p, activePID.i, activePID.d);
        telemetry.addData("Max Power", maxPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        spindexMotor.setPower(0);
    }
}
