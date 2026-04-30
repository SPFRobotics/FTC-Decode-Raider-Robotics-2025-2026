package org.firstinspires.ftc.teamcode.Subsystems.NextFTC;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Depreciated.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.DualColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextTurret;

import java.util.Arrays;
import java.util.Collections;
import java.util.Set;

@Config
public class NextSpindex implements Subsystem {

    public static final NextSpindex INSTANCE = new NextSpindex();

    // Dashboard-tunable position PID gains
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0005;

    // Dashboard-tunable feedforward gains
    // kS: static friction override (constant power in direction of error to break stiction)
    // kV: velocity feedforward (power proportional to target velocity -- usually 0 for position-only)
    public static double kS = 0.24;
    public static double kV = 0.0;

    // Dashboard-tunable positions (degrees)
    public static double posOffset = 10;
    public static double[] intakePos = {0 + posOffset, 120 + posOffset, 240 + posOffset};
    public static double[] outtakePos = {180 + posOffset, 300 + posOffset, 60 + posOffset};

    public static double ballDistanceThreshold = 2.3;
    public static double positionToleranceDeg = 5.0;

    private static final double MAX_VOLTAGE = 3.216;

    public static final String motif21Pattern = "GPP";
    public static final String motif22Pattern = "PGP";
    public static final String motif23Pattern = "PPG";

    private final MotorEx motor = new MotorEx("spindex").reversed().brakeMode();
    private AnalogInput analogEncoder;
    private ControlSystem controlSystem;

    private int index = 0;
    private double error = 0;
    private double targetDeg = 0;

    // Absolute encoder tracking for velocity estimation
    private double prevAbsDeg = 0;
    private long prevTimeNanos = 0;
    private double absVelocityDegPerSec = 0;

    // Cumulative (unwrapped) position tracking for PID
    // This avoids the PID seeing a jump from 359->1 as a huge error
    private double cumulativeDeg = 0;
    private double cumulativeTarget = 0;

    private boolean outtakeMode = false;
    private boolean autoLoadMode = false;
    private boolean autoLaunchMode = false;

    private char[] slotColors = {'E', 'E', 'E'};

    // Auto-launch state
    private enum AutoLaunchState { NEXT_SLOT, WAITFORSPINDEX, LAUNCH }
    private AutoLaunchState autoLaunchState = AutoLaunchState.NEXT_SLOT;
    private final ElapsedTime kickerTimer = new ElapsedTime();
    private int autoLaunchCount = 0;

    // Auto-sort state
    private enum AutoSortState { FIND_NEXT, ROTATING, LAUNCHING, TRY_SHOOT_UNDETECTED, COMPLETE }
    private AutoSortState autoSortState = AutoSortState.FIND_NEXT;
    private int sortPatternIndex = 0;
    private boolean autoSortActive = false;
    private int tryShootSlot = 0;
    private boolean tryingUndetected = false;
    private final ElapsedTime rotateSettleTimer = new ElapsedTime();

    private NextSpindex() {}

    @Override
    public void initialize() {
        analogEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "spindexPos");

        controlSystem = ControlSystem.builder()
                .posPid(kP, kI, kD)
                .basicFF(kV, 0.0, kS)
                .build();
        controlSystem.setGoal(new KineticState(0, 0, 0));

        motor.zeroed();

        // Initialize absolute encoder tracking
        double initDeg = readAbsoluteDegrees();
        prevAbsDeg = initDeg;
        prevTimeNanos = System.nanoTime();
        absVelocityDegPerSec = 0;
        cumulativeDeg = initDeg;
        cumulativeTarget = initDeg;

        // Set the control system goal to current position (no movement on init)
        controlSystem.setGoal(new KineticState(cumulativeDeg, 0, 0));

        index = 0;
        outtakeMode = false;
        autoLoadMode = false;
        autoLaunchMode = false;
        slotColors = new char[]{'E', 'E', 'E'};
        resetAutoSort();
        resetAutoLaunch();
    }

    @Override
    public void periodic() {
        updateAbsoluteEncoder();
        KineticState currentState = new KineticState(cumulativeDeg, absVelocityDegPerSec);
        motor.setPower(controlSystem.calculate(currentState));
    }

    // ---- Absolute Encoder Helpers ----

    /**
     * Reads the raw absolute encoder angle in degrees [0, 360).
     */
    private double readAbsoluteDegrees() {
        return analogEncoder.getVoltage() / MAX_VOLTAGE * 360.0;
    }

    /**
     * Updates the cumulative (unwrapped) position and velocity from the absolute encoder.
     * Called every loop in periodic().
     */
    private void updateAbsoluteEncoder() {
        double currentAbsDeg = readAbsoluteDegrees();
        long currentTimeNanos = System.nanoTime();

        // Compute the wrapped delta (shortest path) from previous reading
        double delta = normalizeDeg(currentAbsDeg - prevAbsDeg);
        cumulativeDeg += delta;

        // Estimate velocity (degrees per second)
        double dtSec = (currentTimeNanos - prevTimeNanos) / 1_000_000_000.0;
        if (dtSec > 0) {
            absVelocityDegPerSec = delta / dtSec;
        }

        prevAbsDeg = currentAbsDeg;
        prevTimeNanos = currentTimeNanos;
    }

    /**
     * Normalizes a degree value to the range [-180, 180).
     */
    private double normalizeDeg(double deg) {
        deg = deg % 360.0;
        if (deg >= 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }

    // ---- Position Control ----

    /**
     * Moves the spindex to the target angle (degrees) using the shortest path.
     * Calculates the wrapped error and sets an absolute cumulative target for the PID.
     */
    public void moveToPos(double targetDegrees) {
        targetDeg = targetDegrees;

        // Compute the shortest-path error from current absolute position to target
        double currentAbsDeg = readAbsoluteDegrees();
        error = normalizeDeg(targetDegrees - currentAbsDeg);

        // Set the cumulative target by adding the error to the current cumulative position
        cumulativeTarget = cumulativeDeg + error;
        controlSystem.setGoal(new KineticState(cumulativeTarget, 0, 0));
    }

    public boolean isBusy() {
        return Math.abs(cumulativeDeg - cumulativeTarget) > positionToleranceDeg;
    }

    // ---- Command Factories ----

    public Command moveTo(double targetDegrees) {
        return new LambdaCommand("SpindexMoveTo(" + targetDegrees + ")")
                .setStart(() -> moveToPos(targetDegrees))
                .setIsDone(() -> !isBusy())
                .requires(this);
    }

    public Command moveToIntake() {
        return new LambdaCommand("SpindexToIntake")
                .setStart(() -> moveToPos(intakePos[index]))
                .setIsDone(() -> !isBusy())
                .requires(this);
    }

    public Command moveToOuttake() {
        return new LambdaCommand("SpindexToOuttake")
                .setStart(() -> moveToPos(outtakePos[index]))
                .setIsDone(() -> !isBusy())
                .requires(this);
    }

    // ---- Index Management ----

    public void addIndex() {
        index = Math.floorMod(index + 1, 3);
    }

    public void subtractIndex() {
        index = Math.floorMod(index - 1, 3);
    }

    public void setIndex(int i) {
        index = i;
    }

    public int getIndex() {
        return index;
    }

    // ---- Slot Color Management ----

    public char[] getSlotColors() {
        return slotColors;
    }

    public void setSlotColor(char color) {
        if (slotColors[index] == 'E') {
            slotColors[index] = color;
        }
    }

    public void clearBall(int idx) {
        slotColors[idx] = 'E';
    }

    public int getIndexOfColor(char color) {
        for (int i = 0; i < 3; i++) {
            if (color == slotColors[i]) {
                return i;
            }
        }
        return -1;
    }

    // ---- Auto-Load ----

    public void autoLoad(ColorFetch colorSensor) {
        double ballDistance = colorSensor.getDistance();
        if (slotColors[index] == 'E' && !isOuttakeing() && !isBusy() && ballDistance < ballDistanceThreshold) {
            setSlotColor(colorSensor.getAverageColor());
        }
        advanceToEmptySlot();
    }

    public void autoLoad(DualColorFetch colorSensor) {
        double[] ballDistances = colorSensor.getDistances();
        float currentHue = 0;

        if (ballDistances[0] <= ballDistanceThreshold) {
            currentHue = colorSensor.getHues()[0];
        }
        if (ballDistances[1] <= ballDistanceThreshold) {
            currentHue = colorSensor.getHues()[1];
        }

        if (currentHue != 0 && slotColors[index] == 'E' && !isBusy()) {
            setSlotColor(colorSensor.getColor(currentHue));
        }
        advanceToEmptySlot();
    }

    public void autoLoadByDistance(DualColorFetch colorSensor) {
        if (slotColors[index] == 'E' && !isBusy() && colorSensor.ballDetected()) {
            slotColors[index] = 'U';
        }
        advanceToEmptySlot();
    }

    private void advanceToEmptySlot() {
        if (isAutoLoading() && slotColors[index] != 'E') {
            for (int i = 0; i < slotColors.length; i++) {
                if (slotColors[i] == 'E') {
                    setIndex(i);
                    break;
                }
            }
        }
    }

    // ---- Auto-Launch ----

    public void autoLaunch(KickerSpindex kicker) {
        if (!autoLaunchMode) return;

        switch (autoLaunchState) {
            case NEXT_SLOT:
                if (autoLaunchCount >= 3) {
                    autoLaunchMode = false;
                    return;
                }
                autoLaunchState = AutoLaunchState.WAITFORSPINDEX;
                break;

            case WAITFORSPINDEX:
                kicker.down();
                if (!isBusy()) {
                    autoLaunchState = AutoLaunchState.LAUNCH;
                    kickerTimer.reset();
                }
                break;

            case LAUNCH:
                if (kickerTimer.milliseconds() < 100) {
                    kicker.up();
                } else if (kickerTimer.milliseconds() < 300) {
                    kicker.down();
                } else {
                    clearBall(index);
                    addIndex();
                    autoLaunchCount++;
                    autoLaunchState = AutoLaunchState.NEXT_SLOT;
                }
                break;
        }
    }

    public void resetAutoLaunch() {
        autoLaunchState = AutoLaunchState.NEXT_SLOT;
        autoLaunchCount = 0;
    }

    // ---- Auto-Sort ----

    public void autoSort(NextOuttake outtake, int motifId, String knownSlotColors) {
        autoSort(outtake, motifId, null, knownSlotColors);
    }

    public void autoSort(NextOuttake outtake, int motifId, NextTurret turret, String knownSlotColors) {
        if (autoSortState == AutoSortState.FIND_NEXT && sortPatternIndex == 0) {
            for (int i = 0; i < 3 && i < knownSlotColors.length(); i++) {
                slotColors[i] = knownSlotColors.charAt(i);
            }
        }
        autoSort(outtake, motifId, turret);
    }

    public void autoSort(NextOuttake outtake, int motifId, NextTurret turret) {
        if (!autoSortActive) return;
        if (motifId < 21 || motifId > 23) motifId = 21;

        String patternStr;
        switch (motifId) {
            case 22: patternStr = motif22Pattern; break;
            case 23: patternStr = motif23Pattern; break;
            default: patternStr = motif21Pattern; break;
        }
        char[] pattern = patternStr.toCharArray();

        switch (autoSortState) {
            case FIND_NEXT:
                if (sortPatternIndex >= pattern.length) {
                    autoSortState = AutoSortState.COMPLETE;
                    return;
                }

                boolean allFilled = slotColors[0] != 'E' && slotColors[1] != 'E' && slotColors[2] != 'E';
                if (!allFilled && sortPatternIndex == 0) {
                    tryShootSlot = 0;
                    tryingUndetected = true;
                    autoSortState = AutoSortState.TRY_SHOOT_UNDETECTED;
                    return;
                }

                char needed = pattern[sortPatternIndex];
                for (int i = 0; i < 3; i++) {
                    if (slotColors[i] == needed) {
                        setMode(true);
                        setIndex(i);
                        rotateSettleTimer.reset();
                        autoSortState = AutoSortState.ROTATING;
                        return;
                    }
                }
                for (int i = 0; i < 3; i++) {
                    if (slotColors[i] != 'E') {
                        setMode(true);
                        setIndex(i);
                        rotateSettleTimer.reset();
                        autoSortState = AutoSortState.ROTATING;
                        return;
                    }
                }
                sortPatternIndex++;
                break;

            case TRY_SHOOT_UNDETECTED:
                if (tryShootSlot >= 3) {
                    autoSortState = AutoSortState.COMPLETE;
                    return;
                }
                setMode(true);
                setIndex(tryShootSlot);
                rotateSettleTimer.reset();
                autoSortState = AutoSortState.ROTATING;
                break;

            case ROTATING:
                double targetRPM = outtake.isFarLocation()
                        ? NextOuttake.farRPM
                        : NextOuttake.closeRPM;
                outtake.setRPM(targetRPM);
                if (!isBusy() && rotateSettleTimer.milliseconds() > 0) {
                    autoSortState = AutoSortState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                targetRPM = outtake.isFarLocation()
                        ? NextOuttake.farRPM
                        : NextOuttake.closeRPM;
                outtake.enableSpindexKickerCycle(true, targetRPM);

                if (outtake.getKickerCycleCount() >= 1) {
                    clearBall(getIndex());
                    outtake.resetKickerCycle();
                    if (tryingUndetected) {
                        tryShootSlot++;
                        autoSortState = AutoSortState.TRY_SHOOT_UNDETECTED;
                    } else {
                        sortPatternIndex++;
                        autoSortState = AutoSortState.FIND_NEXT;
                    }
                }
                break;

            case COMPLETE:
                autoSortActive = false;
                break;
        }
    }

    public void setAutoSortActive(boolean active) {
        autoSortActive = active;
        if (active) {
            sortPatternIndex = 0;
            tryingUndetected = false;
            autoSortState = AutoSortState.FIND_NEXT;
        }
    }

    public void resetAutoSort() {
        autoSortActive = false;
        sortPatternIndex = 0;
        tryingUndetected = false;
        autoSortState = AutoSortState.FIND_NEXT;
    }

    public boolean isAutoSorting() { return autoSortActive; }
    public boolean isAutoSortComplete() { return autoSortState == AutoSortState.COMPLETE; }
    public String getAutoSortStateName() { return autoSortState.name(); }
    public int getSortPatternIndex() { return sortPatternIndex; }

    // ---- Mode & State ----

    public void setMode(boolean outtake) {
        outtakeMode = outtake;
    }

    public boolean isOuttakeing() {
        return outtakeMode;
    }

    public void setAutoLoadMode(boolean x) {
        autoLoadMode = x;
    }

    public boolean isAutoLoading() {
        return autoLoadMode;
    }

    public void setAutoLaunchMode(boolean x) {
        autoLaunchMode = x;
    }

    public boolean isAutoLaunching() {
        return autoLaunchMode;
    }

    // ---- Getters ----

    /**
     * Returns the raw absolute encoder angle in degrees [0, 360).
     */
    public double getPos() {
        return readAbsoluteDegrees();
    }

    /**
     * Returns the unwrapped cumulative position in degrees (used by the PID).
     */
    public double getCumulativePos() {
        return cumulativeDeg;
    }

    /**
     * Returns the normalized absolute position in degrees [0, 360).
     */
    public double getNormAngPos() {
        double norm = cumulativeDeg % 360.0;
        if (norm < 0) norm += 360.0;
        return norm;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return motor.getPower();
    }

    public double getVelocity() {
        return absVelocityDegPerSec;
    }

    public double getAmps() {
        return motor.getMotor().getCurrent(CurrentUnit.AMPS);
    }

    public double getVoltage() {
        return analogEncoder.getVoltage();
    }

    public boolean atTarget(double toleranceDeg) {
        return Math.abs(cumulativeDeg - cumulativeTarget) <= toleranceDeg;
    }

    // ---- Telemetry ----

    public void showTelemetry(Telemetry telemetry) {
        telemetry.addData("Spindex Abs Encoder", "%.1f°", readAbsoluteDegrees());
        telemetry.addData("Spindex Cumulative Pos", "%.1f°", cumulativeDeg);
        telemetry.addData("Spindex Target", "%.1f°", cumulativeTarget);
        telemetry.addData("Spindex Error", "%.1f°", cumulativeDeg - cumulativeTarget);
        telemetry.addData("Spindex Velocity", "%.1f°/s", absVelocityDegPerSec);
        telemetry.addData("Spindex Power", "%.3f", getPower());
        telemetry.addData("Slot Colors", Arrays.toString(slotColors));
    }

    public void showTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Spindex Abs Encoder", "%.1f°", readAbsoluteDegrees());
        telemetry.addData("Spindex Cumulative Pos", "%.1f°", cumulativeDeg);
        telemetry.addData("Spindex Target", "%.1f°", cumulativeTarget);
        telemetry.addData("Spindex Error", "%.1f°", cumulativeDeg - cumulativeTarget);
        telemetry.addData("Spindex Velocity", "%.1f°/s", absVelocityDegPerSec);
        telemetry.addData("Spindex Power", "%.3f", getPower());
        telemetry.addData("Slot Colors", Arrays.toString(slotColors));
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
