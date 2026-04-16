package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextTurret;

//@Disabled
@TeleOp(name = "NextTurret Tuner", group = "Testing")
public class NextTurretTuner extends OpMode {

    NextTurret turret = NextTurret.INSTANCE;

    MultipleTelemetry multiTelemetry;

    @Config
    public static class TunerConfig {
        public static double robotX = 0;
        public static double robotY = 0;
        public static double robotHeading = 0;
        public static boolean blueAlliance = true;
    }

    @Override
    public void init() {
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret.setGoalCoords(TunerConfig.blueAlliance);
        turret.initialize();
    }

    @Override
    public void loop() {
        turret.aimAtGoal(TunerConfig.robotX, TunerConfig.robotY, TunerConfig.robotHeading);
        turret.periodic();

        multiTelemetry.addLine("=== NextTurret Tuner ===");
        multiTelemetry.addLine("--- PID Gains ---");
        multiTelemetry.addData("kP", NextTurret.kP);
        multiTelemetry.addData("kI", NextTurret.kI);
        multiTelemetry.addData("kD", NextTurret.kD);
        multiTelemetry.addLine("--- Inputs ---");
        multiTelemetry.addData("Robot X", TunerConfig.robotX);
        multiTelemetry.addData("Robot Y", TunerConfig.robotY);
        multiTelemetry.addData("Robot Heading (deg)", TunerConfig.robotHeading);
        multiTelemetry.addLine("--- Goal ---");
        multiTelemetry.addData("Goal X", turret.getGoalX());
        multiTelemetry.addData("Goal Y", turret.getGoalY());
        multiTelemetry.addLine("--- Turret State ---");
        multiTelemetry.addData("Target Angle (deg)", turret.getTargetDeg());
        multiTelemetry.addData("Current Angle (deg)", turret.getCurrentAngularPosition());
        multiTelemetry.addData("Current Ticks", turret.getCurrentPosition());
        multiTelemetry.addData("Velocity (ticks/s)", turret.getVelocity());
        multiTelemetry.addData("Motor Power", turret.getPower());
        multiTelemetry.addData("At Target?", !turret.isBusy());
        multiTelemetry.update();
    }
}
