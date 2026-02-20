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

        public static double robotX = 0;
        public static double robotY = 0;
        public static double robotHeading = 0;

        public static boolean manual = false;

        public static double manualGoal = 0;

    }


    public void init(){
        turret = new Turret(hardwareMap, true);

    }

    public void loop(){
        if(!TurretTester.manual) {
            turret.aimAtGoal(TurretTester.robotX, TurretTester.robotY, TurretTester.robotHeading);
        }
        else{
            turret.aimAtGoalManual(TurretTester.manualGoal);
        }

    }

}
