package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.PrintWriter;
import java.util.List;

/**
 * @author 67Demon
 */
public class Turret {

    double goalY;
    double goalX;

    private double lastRobotX;
    private double lastRobotY;
    private double lastRobotHeading;

    double turretLimelightOffset = 0;
    public final double RedGoalX = 133;
    public final double RedGoalY = 135;
    public final double BlueGoalX = 11;
    public final double BlueGoalY = 135;
    public final double ticks = 145.1;
    public final double gearRatio = 135.0 / 32.0;
    public final double limelightTicksPerDegree = (ticks / 360.0) * gearRatio;

    private double initialAngleOffset = 0;
    private double filteredTx = 0;

    private boolean alignment = false;
    private boolean locked = false;
    private double lockedAngleDeg = 0;
    private AlignmentMode state = AlignmentMode.OFF;

    DcMotorEx turret;
    Limelight limelight = null;

    public enum AlignmentMode {
        Limelight,
        Odometry,
        Locked,
        OFF
    }

    @Config
    public static class TurretConfig {
        public static double turretPower = 0.8;

        public static int turretShortLockLine = 322;

        public static int turretSHortLockTri = 335;

        // 1.12, .11, 0, 11.22
        public static double[] pidf = {35, 0.01, 12, 0};

        public static int correctionThresholdTicks = 20;
        public static double limelightAngularOffset = 0.0;
    }

    public Turret(HardwareMap hardwareMap, boolean goalCords) {
        this.turret = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turret.setPositionPIDFCoefficients(25);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setTargetPositionTolerance(3);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setGoalCords(goalCords);
    }

    public Turret(HardwareMap hardwareMap, boolean goalCords, Limelight limelight) {
        this.turret = hardwareMap.get(DcMotorEx.class, "turretMotor");
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

    public void useEncoder(){
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void noEncoder(){
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setAlignmentEnabled(boolean enabled) {
        this.alignment = enabled;
        if (!enabled) {
            state = AlignmentMode.OFF;
            filteredTx = 0;
        }
    }

    public boolean isAlignmentEnabled() {
        return alignment;
    }

    public AlignmentMode getAlignmentMode() {
        return state;
    }

    public void setInitialAngle(double angleDeg) {
        angleDeg = wrapDeg360(angleDeg);
        if (angleDeg > 330) angleDeg -= 360;
        this.initialAngleOffset = angleDeg;
    }

    public double getInitialAngle() {
        return initialAngleOffset;
    }

    /**
     * Locks the turret to a fixed angle (in degrees) regardless of robot heading or position.
     * Can be called standalone (like aimAtGoal) or will be respected by periodic().
     */
    public void lockToAngle(double degrees) {
        locked = true;
        lockedAngleDeg = degrees;
        ensureRunToPositionMode();

        turret.setTargetPosition(degreesToTicks(degrees));
        turret.setPower(TurretConfig.turretPower);
    }

    public void unlockTurret() {
        locked = false;
    }

    public boolean isLocked() {
        return locked;
    }

    private double turretDegToShoot(double robotX, double robotY, double robotHeading) {
        double fieldAngleDeg = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
        double turretDeg = fieldAngleDeg - robotHeading;
        return wrapDeg360(turretDeg);
    }

    public void resetLimelightCorrection() {
        filteredTx = 0;
    }

    public void aimAtGoal(double robotX, double robotY, double robotHeading) {
        ensureRunToPositionMode();

        double targetDeg = turretDegToShoot(robotX, robotY, robotHeading);

        turret.setTargetPosition(degreesToTicks(targetDeg));
        turret.setPower(TurretConfig.turretPower);
    }

    /**
     *
     * @param robotX robot x coord
     * @param robotY robot y coord
     * @param robotHeading robot's heading
     */
    public void periodic(double robotX, double robotY, double robotHeading) {
        this.lastRobotX = robotX;
        this.lastRobotY = robotY;
        this.lastRobotHeading = robotHeading;

        if (locked) {
            state = AlignmentMode.Locked;
            lockToAngle(lockedAngleDeg);
            return;
        }

        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;

        if (!alignment) {
            state = AlignmentMode.OFF;
            filteredTx = 0;
        } else if (result != null && result.isValid()) {
            if (hasShootingTag(result)) {
                state = AlignmentMode.Limelight;
            } else {
                state = AlignmentMode.Odometry;
                filteredTx = 0;
            }
        } else {
            state = AlignmentMode.Odometry;
        }

        switch (state) {
            case Limelight:
                aimWithLimelight(result);
                break;

            case Odometry:
                aimAtGoal(robotX, robotY, robotHeading);
                break;

            case OFF:
                break;
        }
    }

    public void aimWithLimelight(LLResult result) {
        ensureRunToPositionMode();

        Double tagTx = (result != null && result.isValid()) ? getShootingTagTx(result) : null;

        if (tagTx == null) {
            filteredTx = 0;
            turret.setTargetPosition(turret.getCurrentPosition());
            turret.setPower(TurretConfig.turretPower);
            return;
        }

        double adjustedTx = -tagTx + TurretConfig.limelightAngularOffset;
        filteredTx = adjustedTx;

        double currentDeg = getCurrentAngularPosition();
        double targetDeg = currentDeg + adjustedTx;

        turret.setTargetPosition(degreesToTicks(targetDeg));
        turret.setPower(TurretConfig.turretPower);
    }

    private static boolean hasShootingTag(LLResult result) {
        return getShootingTagTx(result) != null;
    }


    private static Double getShootingTagTx(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults == null || fiducialResults.isEmpty()) return null;

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            int id = fr.getFiducialId();
            if (id == 20 || id == 24) return fr.getTargetXDegrees();
        }
        return null;
    }

    private static double wrapDeg360(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    private double limelightOffset() {
        if (limelight == null) {
            return 0;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return filteredTx;
        }

        Double tagTx = getShootingTagTx(result);
        if (tagTx == null) {
            filteredTx = 0;
            return 0;
        }

        double rawTx = -tagTx + TurretConfig.limelightAngularOffset;

        int posError = Math.abs(turret.getCurrentPosition() - turret.getTargetPosition());
        if (posError < TurretConfig.correctionThresholdTicks) {
            filteredTx = rawTx;
        }

        return filteredTx;
    }

    private void ensureRunToPositionMode() {
        if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setTargetPositionTolerance(3);
        }
    }

    public boolean isTurretAtTarget() {
        return !turret.isBusy();
    }

    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }

    public double getCurrentAngularPosition() {
        return turret.getCurrentPosition() / (ticks * gearRatio) * 360.0 + initialAngleOffset;
    }

    /**
     * Converts a physical turret angle to encoder ticks, accounting for
     * the initialAngleOffset (where encoder 0 sits physically).
     * Wraps to ~[-20, 340) physical range to prevent wire tangling.
     */
    private int degreesToTicks(double physicalDeg) {
        physicalDeg = wrapDeg360(physicalDeg);
        if (physicalDeg > 330) physicalDeg -= 360;
        double encoderDeg = physicalDeg - initialAngleOffset;
        return (int) ((encoderDeg / 360.0) * ticks * gearRatio);
    }

    public int getTargetPosition() {
        return turret.getTargetPosition();
    }

    public double getTargetDeg(double robotX, double robotY, double robotHeading) {
        return turretDegToShoot(robotX, robotY, robotHeading);
    }

    public double getGoalX() {
        return goalX;
    }

    public double getGoalY() {
        return goalY;
    }

    /**
     * @param goalCords true for blue, false for red
     */
    private void setGoalCords(boolean goalCords) {
        if (goalCords) {
            goalX = BlueGoalX;
            goalY = BlueGoalY;
        } else {
            goalX = RedGoalX;
            goalY = RedGoalY;
        }
    }

    public void setPower(double power) {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(power);
    }

    public void setLimelightOffset(double x){
        turretLimelightOffset = x;
    }

    public double getVelocity() {
        return turret.getVelocity();
    }

    public void showTelemetry(Telemetry telemetry) {
        telemetry.addLine("------------------------------------------");
        telemetry.addLine("Turret");
        telemetry.addLine("Mode: " + state);
        telemetry.addLine("Alignment Enabled: " + alignment);
        telemetry.addLine("Robot X: " + lastRobotX);
        telemetry.addLine("Robot Y: " + lastRobotY);
        telemetry.addLine("Robot Heading: " + lastRobotHeading);
        telemetry.addLine("Turret Target Degrees: " + getTargetDeg(lastRobotX, lastRobotY, lastRobotHeading));
        telemetry.addLine("Turret Degrees: " + getCurrentAngularPosition());
        telemetry.addLine("Turret Power: " + turret.getPower());
        telemetry.addLine("Filtered Tx: " + filteredTx);
        telemetry.addLine("------------------------------------------");
    }

    public void showTelemetry(MultipleTelemetry telemetry) {
        telemetry.addLine("------------------------------------------------------------------------------------");
        telemetry.addLine("Turret");
        telemetry.addLine("Mode: " + state);
        telemetry.addLine("Alignment Enabled: " + alignment);
        telemetry.addLine("Robot X: " + lastRobotX);
        telemetry.addLine("Robot Y: " + lastRobotY);
        telemetry.addLine("Robot Heading: " + lastRobotHeading);
        telemetry.addLine("Turret Target Degrees: " + getTargetDeg(lastRobotX, lastRobotY, lastRobotHeading));
        telemetry.addLine("Turret Degrees: " + getCurrentAngularPosition());
        telemetry.addLine("Turret Power: " + turret.getPower());
        telemetry.addLine("Filtered Tx: " + filteredTx);
        telemetry.addLine("------------------------------------------------------------------------------------");
    }

    public void log(PrintWriter pen) {
        pen.write("Mode: " + state);
        pen.write("\nAlignment Enabled: " + alignment);
        pen.write("\nRobot X: " + lastRobotX);
        pen.write("\nRobot Y: " + lastRobotY);
        pen.write("\nRobot Heading: " + lastRobotHeading);
        pen.write("\nTurret Target Degrees: " + getTargetDeg(lastRobotX, lastRobotY, lastRobotHeading));
        pen.write("\nTurret Degrees: " + getCurrentAngularPosition());
        pen.write("\nTurret Power: " + turret.getPower());
        pen.write("\nFiltered Tx: " + filteredTx);

        double tx = 0;
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tx = result.getTx();
            }
        }

        pen.write("\nLimelight Offset Angle: " + tx + "\n\n");
    }
}