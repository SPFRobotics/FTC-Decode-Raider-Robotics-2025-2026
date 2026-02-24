package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class AverageTest extends LinearOpMode {
    public static class DistanceSensorTestingConfig{
        public static int loops = 1;
    }
    public void runOpMode() {
        double distances = 0;
        double endTime = 0;
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();

        RevColorSensorV3 colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        waitForStart();
        int i = 0;
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive() && i < DistanceSensorTestingConfig.loops) {
            distances += colorSensorV3.getDistance(DistanceUnit.CM);
            i++;
        }
        endTime = time.milliseconds();
        telemetry.addLine("Average Distance: " + distances/DistanceSensorTestingConfig.loops);
        telemetry.addLine("Loop Time: " + endTime);
        telemetry.update();
    }
}
