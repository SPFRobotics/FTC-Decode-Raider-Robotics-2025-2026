package org.firstinspires.ftc.teamcode.Subsystems.NextFTC;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.Limelight;

import java.util.Collections;
import java.util.List;
import java.util.Set;

@Config
public class NextTurret implements Subsystem {

    public static final NextTurret INSTANCE = new NextTurret();

    // Dashboard-tunable PID gains
    public static double kP = 0.007;
    public static double kI = 0;
    public static double kD = 0.0009;

    // Dashboard-tunable power limit and correction settings
    public static double turretPower = 0.8;
    public static int correctionThresholdTicks = 20;
    public static double limelightAngularOffset = 0.0;
    public static double wrapGracePeriodMs = 200;

    // Preset lock positions (ticks)
    public static int turretShortLockLine = 322;
    public static int turretShortLockTri = 335;
    public static int turretFarLock = 297;

    // Physical constants
    public static final double TICKS = 145.1;
    public static final double GEAR_RATIO = 135.0 / 32.0;

    // Goal field coordinates (inches)
    public static final double RED_GOAL_X = 133;
    public static final double RED_GOAL_Y = 135;
    public static final double BLUE_GOAL_X = 11;
    public static final double BLUE_GOAL_Y = 135;

    public enum AlignmentMode { Limelight, Odometry, Locked, OFF }

    private final MotorEx motor = new MotorEx("turretMotor").reversed().brakeMode();

    private ControlSystem controlSystem;
    private Limelight limelight = null;

    private double goalX = RED_GOAL_X;
    private double goalY = RED_GOAL_Y;
    private int farPipeline = 2;

    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;

    private double initialAngleOffset = 0;
    private double filteredTx = 0;

    private boolean alignment = false;
    private boolean locked = false;
    private double lockedAngleDeg = 0;
    private AlignmentMode state = AlignmentMode.OFF;

    private boolean shortMode = true;
    private int lastPipeline = -1;
    private int targetTagId = -1;
    private int lastTrackedTagId = -1;
    private int targetPositionTicks = 0;

    private final ElapsedTime tagLostTimer = new ElapsedTime();
    private boolean tagLostTimerRunning = false;

    private NextTurret() {}

    @Override
    public void initialize() {
        controlSystem = ControlSystem.builder()
                .posPid(kP, kI, kD)
                .build();
        controlSystem.setGoal(new KineticState(0, 0, 0));

        motor.zeroed();

        alignment = false;
        locked = false;
        filteredTx = 0;
        state = AlignmentMode.OFF;
        lastPipeline = -1;
        lastTrackedTagId = -1;
        tagLostTimerRunning = false;
    }

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }
    public void periodic(double robotX, double robotY, double robotHeading) {
        motor.setPower(controlSystem.calculate(motor.getState()));
        update(robotX, robotY, robotHeading);
    }


    // ---- Robot Pose / Alignment Update ----

    /**
     * Call this every loop from your OpMode (equivalent to the old Turret.periodic()).
     * Stores the robot pose and runs the full alignment state machine.
     */
    public void update(double robotX, double robotY, double robotHeading) {
        this.robotX = robotX;
        this.robotY = robotY;
        this.robotHeading = robotHeading;

        updatePipeline();

        if (locked) {
            state = AlignmentMode.Locked;
            setTargetDegrees(lockedAngleDeg);
            return;
        }

        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;
        Double shootingTagTx = (result != null && result.isValid()) ? getShootingTagTx(result) : null;

        if (!alignment) {
            state = AlignmentMode.OFF;
            filteredTx = 0;
            tagLostTimerRunning = false;
        } else if (shootingTagTx != null) {
            state = AlignmentMode.Limelight;
            tagLostTimerRunning = false;
        } else if (state == AlignmentMode.Limelight) {
            if (!tagLostTimerRunning) {
                tagLostTimer.reset();
                tagLostTimerRunning = true;
            }
            if (tagLostTimer.milliseconds() >= wrapGracePeriodMs) {
                state = AlignmentMode.Odometry;
                filteredTx = 0;
                tagLostTimerRunning = false;
            }
        } else {
            state = AlignmentMode.Odometry;
            filteredTx = 0;
            tagLostTimerRunning = false;
        }

        switch (state) {
            case Limelight:
                if (shootingTagTx != null) aimWithShootingTag(shootingTagTx);
                break;
            case Odometry:
                aimAtGoal(robotX, robotY, robotHeading);
                break;
            case OFF:
                break;
        }
    }

    // ---- Alignment Control ----

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

    // ---- Lock ----

    public void lockToAngle(double degrees) {
        locked = true;
        lockedAngleDeg = degrees;
        setTargetDegrees(degrees);
    }

    public void unlockTurret() {
        locked = false;
    }

    public boolean isLocked() {
        return locked;
    }

    // ---- Aiming ----

    public void aimAtGoal(double robotX, double robotY, double robotHeading) {
        this.robotX = robotX;
        this.robotY = robotY;
        this.robotHeading = robotHeading;
        double targetDeg = turretDegToShoot(robotX, robotY, robotHeading);
        setTargetDegrees(targetDeg);
    }

    public void resetLimelightCorrection() {
        filteredTx = 0;
    }

    // ---- Mode ----

    public void setShortMode(boolean isShort) {
        this.shortMode = isShort;
    }

    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
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
     * @param blueAlliance true for blue alliance, false for red
     */
    public void setGoalCoords(boolean blueAlliance) {
        if (blueAlliance) {
            goalX = BLUE_GOAL_X;
            goalY = BLUE_GOAL_Y;
            farPipeline = 0;
        } else {
            goalX = RED_GOAL_X;
            goalY = RED_GOAL_Y;
            farPipeline = 2;
        }
    }

    public void setLimelight(Limelight limelight) {
        this.limelight = limelight;
    }

    // ---- Private Helpers ----

    private void setTargetDegrees(double degrees) {
        targetPositionTicks = degreesToTicks(degrees);
        controlSystem.setGoal(new KineticState(targetPositionTicks, 0, 0));
    }

    private void updatePipeline() {
        if (limelight == null) return;
        int desired = shortMode ? 1 : farPipeline;
        if (desired != lastPipeline) {
            limelight.setPipeline(desired);
            lastPipeline = desired;
        }
    }

    private void aimWithShootingTag(Double tagTx) {
        if (tagTx == null) {
            filteredTx = 0;
            return;
        }
        double adjustedTx = -tagTx + limelightAngularOffset;

        int posError = (int) Math.abs(motor.getCurrentPosition() - targetPositionTicks);
        if (posError < correctionThresholdTicks) {
            filteredTx = adjustedTx;
        }


        double correctionTicks = (filteredTx / 360.0) * TICKS * GEAR_RATIO;
        targetPositionTicks = (int) (motor.getCurrentPosition() + correctionTicks);
        controlSystem.setGoal(new KineticState(targetPositionTicks, 0, 0));
    }

    private Double getShootingTagTx(LLResult result) {
        List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
        if (fids == null || fids.isEmpty()) {
            lastTrackedTagId = -1;
            return null;
        }
        for (LLResultTypes.FiducialResult fr : fids) {
            int id = fr.getFiducialId();
            if (targetTagId == -1 ? (id == 20 || id == 24) : (id == targetTagId)) {
                lastTrackedTagId = id;
                return fr.getTargetXDegrees();
            }
        }
        lastTrackedTagId = -1;
        return null;
    }

    private double turretDegToShoot(double robotX, double robotY, double robotHeading) {
        double fieldAngle = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
        return wrapDeg360(fieldAngle - robotHeading);
    }

    private int degreesToTicks(double physicalDeg) {
        physicalDeg = wrapDeg360(physicalDeg);
        if (physicalDeg > 330) physicalDeg -= 360;
        double encoderDeg = physicalDeg - initialAngleOffset;
        return (int) ((encoderDeg / 360.0) * TICKS * GEAR_RATIO);
    }

    private static double wrapDeg360(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    // ---- Getters ----

    public double getCurrentAngularPosition() {
        return wrapDeg360(motor.getCurrentPosition() / (TICKS * GEAR_RATIO) * 360.0 + initialAngleOffset);
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getTargetDeg() {
        return turretDegToShoot(robotX, robotY, robotHeading);
    }

    public boolean isBusy() {
        return !controlSystem.isWithinTolerance(
                new KineticState(3, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
    }

    public double getPower() {
        return motor.getPower();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getGoalX() { return goalX; }
    public double getGoalY() { return goalY; }

    // ---- Commands ----

    public Command lockToAngleCommand(double degrees) {
        return new InstantCommand("TurretLockTo(" + degrees + ")", () -> lockToAngle(degrees))
                .requires(this);
    }

    public Command lockShortLine() {
        return new LambdaCommand("TurretShortLine")
                .setStart(() -> lockToAngle(turretShortLockLine))
                .setIsDone(() -> !isBusy())
                .requires(this);
    }

    public Command lockShortTri() {
        return new LambdaCommand("TurretShortTri")
                .setStart(() -> lockToAngle(turretShortLockTri))
                .setIsDone(() -> !isBusy())
                .requires(this);
    }

    public Command lockFar() {
        return new LambdaCommand("TurretFar")
                .setStart(() -> lockToAngle(turretFarLock))
                .setIsDone(() -> !isBusy())
                .requires(this);
    }

    public Command alignCommand(boolean enable) {
        return new InstantCommand("TurretAlign(" + enable + ")", () -> setAlignmentEnabled(enable))
                .requires(this);
    }

    // ---- Telemetry ----

    public void showTelemetry(Telemetry telemetry) {
        telemetry.addLine("------------------------------------------");
        telemetry.addLine("NextTurret");
        telemetry.addLine("Mode: " + state);
        telemetry.addLine("Alignment Enabled: " + alignment);
        telemetry.addLine("Tracking Tag ID: " + lastTrackedTagId);
        telemetry.addLine("Robot X: " + robotX);
        telemetry.addLine("Robot Y: " + robotY);
        telemetry.addLine("Robot Heading: " + robotHeading);
        telemetry.addLine("Target Degrees: " + getTargetDeg());
        telemetry.addLine("Turret Degrees: " + getCurrentAngularPosition());
        telemetry.addLine("Motor Power: " + getPower());
        telemetry.addLine("Filtered Tx: " + filteredTx);
        telemetry.addLine("------------------------------------------");
    }

    public void showTelemetry(MultipleTelemetry telemetry) {
        telemetry.addLine("------------------------------------------");
        telemetry.addLine("NextTurret");
        telemetry.addLine("Mode: " + state);
        telemetry.addLine("Alignment Enabled: " + alignment);
        telemetry.addLine("Tracking Tag ID: " + lastTrackedTagId);
        telemetry.addLine("Robot X: " + robotX);
        telemetry.addLine("Robot Y: " + robotY);
        telemetry.addLine("Robot Heading: " + robotHeading);
        telemetry.addLine("Target Degrees: " + getTargetDeg());
        telemetry.addLine("Turret Degrees: " + getCurrentAngularPosition());
        telemetry.addLine("Motor Power: " + getPower());
        telemetry.addLine("Filtered Tx: " + filteredTx);
        telemetry.addLine("------------------------------------------");
    }

    // ---- Subsystem Interface ----

    @Override
    public Command getDefaultCommand() {
        return null;
    }

    @Override
    public Set<Subsystem> getSubsystems() {
        return Collections.singleton(this);
    }
}
