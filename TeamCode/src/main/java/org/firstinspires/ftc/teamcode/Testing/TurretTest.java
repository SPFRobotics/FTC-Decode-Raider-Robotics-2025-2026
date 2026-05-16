package org.firstinspires.ftc.teamcode.Testing;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextTurret;
@Disabled
@TeleOp (name="Turret Test")
public class TurretTest extends OpMode {
    NextTurret turret = NextTurret.INSTANCE;
    @Config
    public static class TurretTester{

        public static double robotX = 0;
        public static double robotY = 0;
        public static double robotHeading = 0;

        public static boolean manual = false;

        public static boolean goal = true;

        public static double manualGoal = 35;

    }
    public void init(){
        turret.setGoalCoords(TurretTester.goal);
        turret.initialize(hardwareMap);



    }

    public void stop(){

    }

    public void loop(){
        if(!TurretTester.manual) {
            turret.aimAtGoal(TurretTester.robotX, TurretTester.robotY, TurretTester.robotHeading);
            double targetDeg = turret.getTargetDeg();
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
            //turret.aimAtGoalManual(TurretTester.manualGoal);
            telemetry.addData("Mode", "Manual");
            telemetry.addLine("--- Turret ---");
            telemetry.addData("Manual Goal (deg)", TurretTester.manualGoal);
        }
        turret.periodic();
        telemetry.addData("TurretConfig.ticks", NextTurret.TICKS);
        telemetry.addData("TurretConfig.gearRatio", NextTurret.GEAR_RATIO);
        telemetry.addData("TurretConfig.turretPower", NextTurret.turretPower);
        telemetry.addLine("--- Motor ---");
        telemetry.addData("Current Ticks", turret.getCurrentPosition());
        telemetry.addData("At Target?", !turret.isBusy());
        telemetry.update();
    }

}
