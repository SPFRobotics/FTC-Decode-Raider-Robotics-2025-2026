package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import com.bylazar.telemetry.PanelsTelemetry;

@Disabled
public class OuttakeTesting extends LinearOpMode {
    private TelemetryManager telemetryM;
    PrintWriter pen = new PrintWriter("/sdcard/outtake.txt");

    public OuttakeTesting() throws FileNotFoundException {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Outtake outtake = new Outtake(hardwareMap);
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();
        telemetry.setMsTransmissionInterval(16);
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                outtake.setRPM(3200);
            }
            else if (gamepad1.dpad_down) {
                outtake.setRPM(2700);
            }
            else if (gamepad1.ps){
                outtake.setRPM(0);
            }
            pen.write((int)runtime.milliseconds() + ":" + (int)outtake.getRPM() + "\n");
            telemetryM.addData("RPM", outtake.getRPM());
            telemetryM.debug("RPM", outtake.getRPM());
            telemetry.addData("RPM", outtake.getRPM());
            telemetryM.update();
            telemetry.update();
        }
        pen.close();
    }
}
