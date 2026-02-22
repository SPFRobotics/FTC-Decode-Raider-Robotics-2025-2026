package org.firstinspires.ftc.teamcode.Resources;

import static org.firstinspires.ftc.teamcode.Testing.TurretTest.TurretTester.robotHeading;
import static org.firstinspires.ftc.teamcode.Testing.TurretTest.TurretTester.robotX;
import static org.firstinspires.ftc.teamcode.Testing.TurretTest.TurretTester.robotY;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.intellij.lang.annotations.JdkConstants;

public class ThreadedTurret {

    double goalY;
    double goalX;
    DcMotor turret;
    public TurretThread turretThread = null;

    private volatile double latestRobotX;
    private volatile double latestRobotY;
    private volatile double latestRobotHeading;
    private volatile double computedTargetDeg;
    private volatile boolean inputsUpdated = false;

    @Config
    public static class TurretConfig{

        static double RedGoalX = 132;
        static double RedGoalY = 136;
        static double BlueGoalX = 12;
        static double BlueGoalY = 136;

        public static double ticks = 145.1;
        public static double gearRatio = 135/31.0;
        public static double turretPower = 1;
    }

    public class TurretThread extends Thread {
        private volatile boolean running = true;

        @Override
        public void run() {
            while (running) {
                if (inputsUpdated) {
                    double x = latestRobotX;
                    double y = latestRobotY;
                    double heading = latestRobotHeading;
                    inputsUpdated = false;

                    computedTargetDeg = turretDegToShoot(x, y, heading);
                }

                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    break;
                }
            }
        }

        public void stopThread() {
            running = false;
            this.interrupt();
        }
    }

    public ThreadedTurret(HardwareMap hardwareMap, boolean goalCords){
        this.turret = hardwareMap.get(DcMotor.class, "turretMotor");
        turretThread = new TurretThread();
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setDirection(DcMotor.Direction.REVERSE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setGoalCords(goalCords);

        turretThread.start();
    }

    private double turretDegToShoot(double robotX, double robotY, double robotHeading) {
        double fieldAngleDeg = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
        double turretDeg = fieldAngleDeg - robotHeading;
        return wrapDeg360(turretDeg);
    }

    public void updateRobotPose(double robotX, double robotY, double robotHeading) {
        latestRobotX = robotX;
        latestRobotY = robotY;
        latestRobotHeading = robotHeading;
        inputsUpdated = true;
    }

    public void aimAtGoal() {
        double targetDeg = computedTargetDeg;
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

    public double getTargetDeg() {
        return computedTargetDeg;
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

    public void setPower(double power){
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(power);
    }

    public void stopThread() {
        if (turretThread != null) {
            turretThread.stopThread();
        }
    }
}
