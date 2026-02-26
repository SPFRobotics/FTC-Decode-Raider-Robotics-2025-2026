package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretConfig.pidf;
import static org.firstinspires.ftc.teamcode.Testing.TurretTest.TurretTester.robotHeading;
import static org.firstinspires.ftc.teamcode.Testing.TurretTest.TurretTester.robotX;
import static org.firstinspires.ftc.teamcode.Testing.TurretTest.TurretTester.robotY;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.Unit;
import org.intellij.lang.annotations.JdkConstants;

public class Turret {

    double goalY;
    double goalX;
    public final double RedGoalX = 132;
    public final double RedGoalY = 136;
    public final double BlueGoalX = 12;
    public final double BlueGoalY = 136;
    public final double ticks = 145.1;
    public final double gearRatio = 135/32.0;

    DcMotorEx turret;

    @Config
    public static class TurretConfig{
        public static double turretPower = 1;

        //1.12, .11, 0, 11.22
        public static double[] pidf = {1.62, 0.16, 0, 16.22};
    }

    public Turret(HardwareMap hardwareMap, boolean goalCords){
        this.turret = hardwareMap.get(DcMotorEx.class, "turretMotor");
        //turret.setVelocityPIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]);
        turret.setPositionPIDFCoefficients(25);
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
        int targetTicks = (int) ((targetDeg / 360.0) * ticks * gearRatio);

        turret.setTargetPosition(targetTicks);
        turret.setPower(TurretConfig.turretPower);
    }


    public void aimAtGoalManual(double manualGoal){
        double targetDeg = manualGoal;
        int targetTicks = (int) ((targetDeg / 360.0) * ticks * gearRatio);

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

    public double getCurrentAngularPosition(){
        return turret.getCurrentPosition() / 360.0 * ticks * gearRatio;
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
            goalX = BlueGoalX;
            goalY = BlueGoalY;

        }else{
            goalX = RedGoalX;
            goalY = RedGoalY;
        }
    }

    public void setPower(double power){
        //turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(power);
    }

    public double getVelocity(){
        return turret.getVelocity();
    }

    //Telemetry Blocks
    public void showTelemetry(Telemetry telemetry, double robotX, double robotY, double robotHeading){
        telemetry.addLine("------------------------------------------");
        telemetry.addLine("Turret");
        telemetry.addLine("Robot X: " + robotX);
        telemetry.addLine("Robot Y: " + robotY);
        telemetry.addLine("Robot Heading: " + robotHeading);
        telemetry.addLine("Turret Target Degrees: " + getTargetDeg(robotX, robotY, robotHeading));
        telemetry.addLine("Turret Power: " + turret.getPower());
        telemetry.addLine("------------------------------------------");
    }

    public void showTelemetry(MultipleTelemetry telemetry, double robotX, double robotY, double robotHeading){
        telemetry.addLine("------------------------------------------------------------------------------------");
        telemetry.addLine("Turret");
        telemetry.addLine("Robot X: " + robotX);
        telemetry.addLine("Robot Y: " + robotY);
        telemetry.addLine("Robot Heading: " + robotHeading);
        telemetry.addLine("Turret Target Degrees: " + getTargetDeg(robotX, robotY, robotHeading));
        telemetry.addLine("Turret Power: " + turret.getPower());
        telemetry.addLine("------------------------------------------------------------------------------------");
    }
}
