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


@TeleOp(name = "Spindex PID Tuning", group = "Testing")
public class SpindexTuning extends OpMode {


    @Config
    public static class SpindexTuningConfig {
        public static double kP = 10.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double maxPower = 0.5;
    }


    private DcMotorEx spindexMotor;
    private static final double TICKS_PER_REV = 537.7;

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

        spindexMotor.setTargetPosition(0);
        spindexMotor.setPower(maxPower);

        int currentPos = spindexMotor.getCurrentPosition();
        int error = -currentPos; // target (0) minus current
        double currentDeg = (currentPos / TICKS_PER_REV) * 360.0;
        double motorPower = spindexMotor.getPower();
        PIDCoefficients activePID = spindexMotor.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Position (ticks)", currentPos);
        telemetry.addData("Position (deg)", "%.1f", currentDeg);
        telemetry.addData("Error (ticks)", error);
        telemetry.addData("Motor Power", "%.3f", motorPower);
        telemetry.addData("At Home?", !spindexMotor.isBusy());
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
