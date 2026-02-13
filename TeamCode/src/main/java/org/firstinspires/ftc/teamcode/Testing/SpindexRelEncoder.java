package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="SpindexRelEncoder")
public class SpindexRelEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx spindex = hardwareMap.get(DcMotorEx.class, "spindex");
        spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Pos", spindex.getCurrentPosition());
            telemetry.update();
        }
    }
}
