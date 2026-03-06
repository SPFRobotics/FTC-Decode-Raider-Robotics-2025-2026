package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    double goalY;
    double goalX;

    double turretLimelightOffset = 0;
    private double sotmDistance = 0;
    public final double RedGoalX = 133;
    public final double RedGoalY = 135;
    public final double BlueGoalX = 11;
    public final double BlueGoalY = 135;
    public final double ticks = 145.1;
    public final double gearRatio = 135/32.0;

    private double initialAngleOffset = 0;
    private double filteredTx = 0;

    DcMotorEx turret;

    Limelight limelight = null;

    @Config
    public static class TurretConfig{
        public static double turretPower = 1;

        //1.12, .11, 0, 11.22
        public static double[] pidf = {35, 0.01, 12, 0};

        public static int correctionThresholdTicks = 20;

        public static double sotmForwardScale = 0.001;
        public static double sotmLateralScale = 0.01;
        public static double sotmMaxForwardScale = 1.0;
        public static double sotmMaxLateralScale = 1.0;
    }

    public Turret(HardwareMap hardwareMap, boolean goalCords){
        this.turret = hardwareMap.get(DcMotorEx.class, "turretMotor");
        //turret.setVelocityPIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]);
        turret.setPositionPIDFCoefficients(25);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setTargetPositionTolerance(3);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setGoalCords(goalCords);
    }

    public Turret(HardwareMap hardwareMap, boolean goalCords, Limelight limelight){
        this.turret = hardwareMap.get(DcMotorEx.class, "turretMotor");
        //turret.setVelocityPIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]);
        turret.setPositionPIDFCoefficients(25);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setTargetPositionTolerance(3);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setGoalCords(goalCords);

        this.limelight = limelight;
    }


    public void setInitialAngle(double angleDeg) {
        this.initialAngleOffset = angleDeg;
    }

    public double getInitialAngle() {
        return initialAngleOffset;
    }

    private double turretDegToShoot(double robotX, double robotY, double robotHeading) {

        turretLimelightOffset = limelightOffset();

        double fieldAngleDeg = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

        double turretDeg = fieldAngleDeg - robotHeading + turretLimelightOffset;

        return wrapDeg360(turretDeg);
    }

    public void resetLimelightCorrection() {
        filteredTx = 0;
    }

    public void aimAtGoal(double robotX, double robotY, double robotHeading) {
        double targetDeg = turretDegToShoot(robotX, robotY, robotHeading);
        targetDeg += targetDeg > 34 ? -360 : 0;
        int targetTicks = (int) (((targetDeg - initialAngleOffset) / 360.0) * ticks * gearRatio);

        turret.setTargetPosition(targetTicks);
        turret.setPower(TurretConfig.turretPower);
    }


    public void aimAtGoal(double robotX, double robotY, double robotHeading,
                          double velX, double velY) {
        double rawAngle = Math.atan2(goalY - robotY, goalX - robotX);
        double rawDistance = Math.hypot(goalX - robotX, goalY - robotY);

        double cos = Math.cos(-rawAngle);
        double sin = Math.sin(-rawAngle);
        double forwardVel = velX * cos - velY * sin;
        double lateralVel = velX * sin + velY * cos;

        double fwdScale = clamp(
                rawDistance * TurretConfig.sotmForwardScale,
                -TurretConfig.sotmMaxForwardScale, TurretConfig.sotmMaxForwardScale);
        double latScale = clamp(
                rawDistance * TurretConfig.sotmLateralScale,
                -TurretConfig.sotmMaxLateralScale, TurretConfig.sotmMaxLateralScale);

        double scaledFwd = fwdScale * forwardVel;
        double scaledLat = latScale * lateralVel;

        double cos2 = Math.cos(rawAngle);
        double sin2 = Math.sin(rawAngle);
        double adjustX = scaledFwd * cos2 - scaledLat * sin2;
        double adjustY = scaledFwd * sin2 + scaledLat * cos2;

        double newGoalX = goalX - adjustX;
        double newGoalY = goalY - adjustY;

        sotmDistance = Math.hypot(newGoalX - robotX, newGoalY - robotY);

        turretLimelightOffset = limelightOffset();
        double fieldAngleDeg = Math.toDegrees(Math.atan2(newGoalY - robotY, newGoalX - robotX));
        double turretDeg = fieldAngleDeg - robotHeading + turretLimelightOffset;
        double targetDeg = wrapDeg360(turretDeg);
        targetDeg += targetDeg > 360 ? -360 : 0;

        int targetTicks = (int) (((targetDeg - initialAngleOffset) / 360.0) * ticks * gearRatio);
        turret.setTargetPosition(targetTicks);
        turret.setPower(TurretConfig.turretPower);
    }

    public double getSOTMDistance() {
        return sotmDistance;
    }

    public void aimAtGoalManual(double manualGoal){
        double targetDeg = manualGoal;
        int targetTicks = (int) (((targetDeg - initialAngleOffset) / 360.0) * ticks * gearRatio);

        turret.setTargetPosition(targetTicks);
        turret.setPower(TurretConfig.turretPower);
    }


    private static double wrapDeg360(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double limelightOffset(){
        if (limelight == null) {
            return 0;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return filteredTx;
        }

        double rawTx = result.getTx();

        // Only update when the turret is near its target; during transit
        // tx reflects the position gap, not odometry error.
        int posError = Math.abs(turret.getCurrentPosition() - turret.getTargetPosition());
        if (posError < TurretConfig.correctionThresholdTicks) {
            filteredTx = rawTx;
        }

        return filteredTx;
    }



    public boolean isTurretAtTarget() {
        return !turret.isBusy();
    }

    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }

    public double getCurrentAngularPosition(){
        return turret.getCurrentPosition() / (ticks * gearRatio) * 360.0;
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
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        telemetry.addLine("Turret Degrees: " + (turret.getCurrentPosition()/537.7*360)%360);
        telemetry.addLine("Turret Power: " + turret.getPower());
        telemetry.addLine("------------------------------------------------------------------------------------");
    }
}
