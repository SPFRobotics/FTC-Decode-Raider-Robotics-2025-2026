package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@DIsabled
public class BoreEncoder extends LinearOpMode {
    public void runOpMode(){
        DcMotor encoder = hardwareMap.get(DcMotor.class, "encoder");
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashTelemetry = dash.getTelemetry();
        telemetry.setMsTransmissionInterval(16);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Pos", encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
