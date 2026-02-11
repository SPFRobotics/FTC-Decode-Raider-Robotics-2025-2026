package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class SpindexTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx spindex = hardwareMap.get(DcMotorEx.class, "spindex");
        AnalogInput absEncoder = hardwareMap.get(AnalogInput.class, "spindexPos");

        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindex.setDirection(DcMotorSimple.Direction.REVERSE);

        FtcDashboard dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();

        telemetry.setMsTransmissionInterval(1);

        final int[] POSITIONS = {0, 120, 240};
        int index = 0;

        waitForStart();
        double absEncoderPos = (absEncoder.getVoltage()/3.3*360);
        double offset = absEncoderPos;

        while (opModeIsActive()){
            double target = POSITIONS[index];
            absEncoderPos = absEncoder.getVoltage()/3.3*360;
            double relPos = Math.floorMod((int)(((spindex.getCurrentPosition()/537.7*360)+offset)), 360);
            double error = AngleUnit.normalizeDegrees(target - relPos);
            double ticksError = error/537.7*360;

            spindex.setTargetPosition((int)(spindex.getCurrentPosition()+ticksError));
            spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindex.setVelocityPIDFCoefficients(8, 0.325, 0, 0);
            spindex.setPower(1);

            if (gamepad1.aWasPressed()){
                index = (index+1)%3;
            }

            //Telemetry
            telemetry.addLine("Absolute Encoder Position: " + absEncoderPos);
            telemetry.addLine("Relative Encoder Angular Position: " + relPos);
            telemetry.addLine("Absolute Encoder Voltage: " + absEncoder.getVoltage());
            telemetry.addLine("Relative Encoder Position: " + spindex.getCurrentPosition());
            telemetry.addLine("Possible Offset In Encoder Counts: " + offset);
            telemetry.addLine("Index: " + Integer.toString(index));
            telemetry.update();
        }
    }
}
