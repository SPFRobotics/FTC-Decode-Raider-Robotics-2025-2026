package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp (name="Turret Test")
public class TurretTest extends OpMode {
    Turret turret = null;
    @Config
    public static class TurretTester{

        public static double robotX = 72;
        public static double robotY = 72;
        public static double robotHeading = 90;

        public static boolean manual = false;

        public static boolean goal = true;

        public static double manualGoal = 35;

    }


    public void init(){
        turret = new Turret(hardwareMap, TurretTester.goal);



    }

    public void loop(){
        if(!TurretTester.manual) {
            //turret.aimAtGoal(TurretTester.robotX, TurretTester.robotY, TurretTester.robotHeading);

            double targetDeg = turret.getTargetDeg(TurretTester.robotX, TurretTester.robotY, TurretTester.robotHeading);
            telemetry.addData("Mode", "Auto-Aim");
            telemetry.addLine("--- Inputs ---");
            telemetry.addData("Robot X", TurretTester.robotX);
            telemetry.addData("Robot Y", TurretTester.robotY);
            telemetry.addData("Robot Heading (deg)", TurretTester.robotHeading);
            telemetry.addLine("--- Goal ---");
            telemetry.addData("Goal X", turret.getGoalX());
            telemetry.addData("Goal Y", turret.getGoalY());
            telemetry.addLine("--- Turret ---");
            telemetry.addData("Target Angle (deg)", targetDeg);
        }
        else{
            turret.aimAtGoalManual(TurretTester.manualGoal);

            telemetry.addData("Mode", "Manual");
            telemetry.addLine("--- Turret ---");
            telemetry.addData("Manual Goal (deg)", TurretTester.manualGoal);
        }

        telemetry.addData("Target Ticks", turret.getTargetPosition());
        telemetry.addData("Current Ticks", turret.getCurrentPosition());
        telemetry.addData("Error (ticks)", turret.getTargetPosition() - turret.getCurrentPosition());
        telemetry.addData("At Target?", turret.isTurretAtTarget());
        telemetry.update();
    }

}
