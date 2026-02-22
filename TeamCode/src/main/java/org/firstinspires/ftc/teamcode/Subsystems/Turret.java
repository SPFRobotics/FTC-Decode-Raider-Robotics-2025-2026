package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Testing.TurretTest.TurretTester.robotHeading;
import static org.firstinspires.ftc.teamcode.Testing.TurretTest.TurretTester.robotX;
import static org.firstinspires.ftc.teamcode.Testing.TurretTest.TurretTester.robotY;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    double goalY;
    double goalX;
    DcMotor turret;
    @Config
    public static class TurretConfig{

        static double RedGoalX = 132;
        static double RedGoalY = 136;
        static double BlueGoalX = 12;
        static double BlueGoalY = 136;

        public static double ticks = 537.7;
        public static double gearRatio = 4.5;
        public static double turretPower = 1;
    }
    //@param goalCords True for blue, false for red

    public Turret(HardwareMap hardwareMap, boolean goalCords){
        this.turret = hardwareMap.get(DcMotor.class, "turretMotor");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setDirection(DcMotor.Direction.REVERSE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setGoalCords(goalCords);
    }

        private double turretDegToShoot(double robotX, double robotY, double robotHeading) {

            double fieldAngleDeg = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

            double turretDeg = fieldAngleDeg - robotHeading;

            return wrapDeg360(turretDeg);
        }
    public void aimAtGoal(double robotX, double robotY, double robotHeading) {
        double targetDeg = turretDegToShoot(robotX, robotY, robotHeading);
        int targetTicks = (int) ((targetDeg / 360.0) * TurretConfig.ticks * TurretConfig.gearRatio);

        turret.setTargetPosition(targetTicks);
        turret.setPower(TurretConfig.turretPower);
    }


    public void aimAtGoalManual(double manualGoal){

        double targetDeg = manualGoal;
        int targetTicks = (int) ((targetDeg / 360.0) * TurretConfig.ticks * TurretConfig.gearRatio);

        turret.setTargetPosition(targetTicks);
        turret.setPower(TurretConfig.turretPower);


    }


    private static double wrapDeg360(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    public boolean isTurretAtTarget() {
        return !turret.isBusy();
    }

    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }

    public int getTargetPosition() {
        return turret.getTargetPosition();
    }

    public double getTargetDeg(double robotX, double robotY, double robotHeading) {
        return turretDegToShoot(robotX, robotY, robotHeading);
    }

    public double getGoalX() { return goalX; }
    public double getGoalY() { return goalY; }

    //@param goalCords True for blue, false for red
    private void setGoalCords(boolean goalCords){

        if(goalCords){
            goalX = TurretConfig.BlueGoalX;
            goalY = TurretConfig.BlueGoalY;

        }else{
            goalX = TurretConfig.RedGoalX;
            goalY = TurretConfig.RedGoalY;
        }

    }


}
