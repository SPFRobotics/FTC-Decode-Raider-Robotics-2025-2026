package org.firstinspires.ftc.teamcode.Resources;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


public class VERYheavilyInspiredSpindex {

    // ========================================================================
    // FSM STATES
    // ========================================================================
    public enum SpindexState {
        INTAKE_STATIC,
        SWITCHING_CHAMBERS,
        SHOOTING
    }

    // ========================================================================
    // MOTIF ENUM
    // ========================================================================
    public enum Motif {
        GPP,
        PGP,
        PPG,
    }

    // ========================================================================
    // FTC DASHBOARD CONFIG
    // ========================================================================
    @Config
    public static class SpindexConfig {
        // P-controller
        public static double maxPower = 1.0;
        public static double threshold = 63.75;
        public static double tolerance = 5.0;

        // Chamber positions (degrees) -- proven existing values
        public static double[] intakePos  = {2, 122, 242};
        public static double[] outtakePos = {182, 302, 62};

        // Colour / distance sensor thresholds
        public static double ballDistanceThreshold = 3.0;
        public static double ballReleaseThreshold  = 4.0;
        public static long   detectTimeMs          = 10;

        // Auto-shoot timing (seconds)
        public static double kickUpDuration      = 0.3;
        public static double kickRetractDelay    = 0.2;
        public static double sorterSettleTime    = 0.5;
        public static double rpmThresholdOffset  = 100;
        public static double modeToggleWaitTime  = 0.75;

        // Auto-launch delay (ms)
        public static double launchTime = 900;
    }

    // ========================================================================
    // HARDWARE (owned)
    // ========================================================================
    private final DcMotorEx   spindexMotor;
    private final AnalogInput spindexPos;

    // ========================================================================
    // SUBSYSTEM DEPENDENCIES (injected)
    // ========================================================================
    private final KickerSpindex kicker;
    private final ColorFetch    colorSensor;
    private final Outtake       outtake;

    // ========================================================================
    // FSM
    // ========================================================================
    private SpindexState state = SpindexState.INTAKE_STATIC;

    // ========================================================================
    // CHAMBER TRACKING  (fixed arrays -- index == physical chamber)
    // ========================================================================
    private int       currentChamber = 0;
    private final char[]    chamberColors = {'E', 'E', 'E'};
    private final boolean[] slotStatus    = {false, false, false};

    // ========================================================================
    // POSITION CONTROL
    // ========================================================================
    private double targetPos  = 0;
    private double currentPos = 0;
    private double error      = 0;
    private boolean outtakeMode    = false;
    private boolean autoLoadMode   = false;
    private boolean autoLaunchMode = false;

    // ========================================================================
    // MOTIF
    // ========================================================================
    public Motif currentMotif = Motif.GPP;

    // ========================================================================
    // TIMED COLOUR DETECTION  (debounce + latch)
    // ========================================================================
    private long    colorStartTime = 0;
    private boolean colorActive    = false;
    private boolean ballLatched    = false;

    // ========================================================================
    // AUTO-SHOOT SEQUENCE
    // ========================================================================
    private int          autoShootState = 0;
    private final ElapsedTime autoShootTimer = new ElapsedTime();
    private int          shotsComplete  = 0;

    // ========================================================================
    // AUTO-LAUNCH
    // ========================================================================
    private boolean ballFound = false;
    private final ElapsedTime autoLaunchTimer = new ElapsedTime();

    // ========================================================================
    // COMPATIBILITY FIELDS
    // ========================================================================
    private boolean terminate     = false;
    private double  threadLoopTime = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public VERYheavilyInspiredSpindex(HardwareMap hardwareMap,
                                      KickerSpindex kicker,
                                      ColorFetch colorSensor,
                                      Outtake outtake) {
        this.kicker      = kicker;
        this.colorSensor = colorSensor;
        this.outtake     = outtake;

        spindexMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spindexPos = hardwareMap.get(AnalogInput.class, "spindexPos");

        currentChamber = 0;
        state          = SpindexState.INTAKE_STATIC;
        currentMotif   = Motif.GPP;
        targetPos      = SpindexConfig.intakePos[0];
    }

    // ========================================================================
    // POSITION CONTROL -- PIECEWISE P-CONTROLLER
    // ========================================================================


    private void updatePController() {
        currentPos = spindexPos.getVoltage() / 3.3 * 360.0;

        error = AngleUnit.normalizeDegrees(targetPos - currentPos);

        double sign = Math.signum(error);
        double kp   = SpindexConfig.maxPower / SpindexConfig.threshold;

        if (Math.abs(error) > SpindexConfig.threshold) {
            spindexMotor.setPower(SpindexConfig.maxPower * sign);
        } else if (Math.abs(error) > SpindexConfig.tolerance) {
            spindexMotor.setPower(error * kp);
        } else {
            spindexMotor.setPower(0);
        }

        if (state == SpindexState.SWITCHING_CHAMBERS
                && Math.abs(error) <= SpindexConfig.tolerance) {
            state = SpindexState.INTAKE_STATIC;
        }
    }

    // ========================================================================
    // CHAMBER POSITION HELPERS
    // ========================================================================

    private double intakePosition(int chamber) {
        return SpindexConfig.intakePos[chamber];
    }

    private double outtakePosition(int chamber) {
        return SpindexConfig.outtakePos[chamber];
    }

    private double positionForChamber(int chamber) {
        return outtakeMode ? outtakePosition(chamber) : intakePosition(chamber);
    }


    private int nextChamber(int c) {
        if (c == 0) return 2;
        if (c == 2) return 1;
        return 0;  // 1 -> 0
    }


    private int prevChamber(int c) {
        if (c == 0) return 1;
        if (c == 1) return 2;
        return 0;  // 2 -> 0
    }

    // ========================================================================
    // AUTO INTAKE -- DEBOUNCED COLOUR DETECTION
    // ========================================================================


    private void autoIntakeColorCheck() {
        double distance = colorSensor.getDistance();
        char   detected = colorSensor.getColor();

        // Release latch when ball clears the sensor
        if (distance > SpindexConfig.ballReleaseThreshold) {
            ballLatched = false;
        }

        // Nothing useful or already latched -- reset debounce
        if (detected == 'E' || detected == 0
                || distance > SpindexConfig.ballDistanceThreshold
                || ballLatched) {
            colorActive    = false;
            colorStartTime = 0;
            return;
        }

        // Start / continue debounce timer
        if (!colorActive) {
            colorActive    = true;
            colorStartTime = System.currentTimeMillis();
        }

        // Detection persisted long enough -- accept the ball
        if (System.currentTimeMillis() - colorStartTime >= SpindexConfig.detectTimeMs) {
            if (chamberColors[currentChamber] == 'E') {
                chamberColors[currentChamber] = detected;
                slotStatus[currentChamber]    = true;
                ballLatched = true;

                // Rotate to the next empty chamber
                int next = findNextEmpty(currentChamber);
                if (next != -1 && next != currentChamber) {
                    currentChamber = next;
                    state = SpindexState.SWITCHING_CHAMBERS;
                    targetPos = intakePosition(currentChamber);
                }
            }

            colorActive    = false;
            colorStartTime = 0;
        }
    }


    private int findNextEmpty(int from) {
        for (int offset = 1; offset <= 3; offset++) {
            int idx = (from + offset) % 3;
            if (chamberColors[idx] == 'E') return idx;
        }
        return -1;
    }

    // ========================================================================
    // MOTIF & AUTO-ALIGN
    // ========================================================================

    public void cycleMotif() {
        switch (currentMotif) {
            case GPP: currentMotif = Motif.PGP; break;
            case PGP: currentMotif = Motif.PPG; break;
            case PPG: currentMotif = Motif.GPP; break;
        }
    }



    public void autoAlignChamberColors() {
        int greenIdx = indexOfColor('G', true);
        if (greenIdx == -1) {
            autoShootState = 0;
            return;
        }

        switch (currentMotif) {
            case GPP:
                currentChamber = greenIdx;
                break;
            case PGP:
                currentChamber = prevChamber(greenIdx);
                break;
            case PPG:
                currentChamber = prevChamber(prevChamber(greenIdx));
                break;
        }

        targetPos = outtakePosition(currentChamber);
        autoShootState = 0;
    }


    private int indexOfColor(char desiredColor, boolean returnNextBest) {
        for (int i = 0; i < 3; i++) {
            if (chamberColors[i] == desiredColor) return i;
        }
        if (returnNextBest) {
            for (int i = 0; i < 3; i++) {
                if (chamberColors[i] != 'E') return i;
            }
        }
        return -1;
    }

    // ========================================================================
    // AUTO-SHOOT SEQUENCE
    // ========================================================================


    public void startShootingSequence() {
        autoShootTimer.reset();
        autoShootState = -1;
        shotsComplete  = 0;
        state          = SpindexState.SHOOTING;
        outtakeMode    = true;
        autoAlignChamberColors();
    }


    private void advanceToNextLoadedChamber() {
        int start = currentChamber;
        currentChamber = nextChamber(currentChamber);
        targetPos = outtakePosition(currentChamber);
    }


    private void updateAutoShootSequence(Gamepad gamepad) {
        switch (autoShootState) {

            case -1: // Still aligning (autoAlignChamberColors sets to 0)
                break;

            // ---- mode-toggle wait + first rotation ----
            case 0:
                if (autoShootTimer.seconds() >= SpindexConfig.modeToggleWaitTime) {
                    targetPos = outtakePosition(currentChamber);
                    autoShootTimer.reset();
                    autoShootState = 1;
                }
                break;

            // ======================= BALL 1 =======================
            case 1: // Wait for sorter to settle at outtake position
                if (autoShootTimer.seconds() >= SpindexConfig.sorterSettleTime
                        && atTarget()) {
                    autoShootTimer.reset();
                    autoShootState = 2;
                }
                break;

            case 2: // Wait for flywheel RPM
                if (outtake.getRPM() >= Outtake.OuttakeConfig.closeRPM
                        - SpindexConfig.rpmThresholdOffset) {
                    kicker.up();
                    autoShootTimer.reset();
                    autoShootState = 3;
                }
                break;

            case 3: // Ball in flight
                if (autoShootTimer.seconds() >= SpindexConfig.kickUpDuration) {
                    kicker.down();
                    chamberColors[currentChamber] = 'E';
                    slotStatus[currentChamber]    = false;
                    shotsComplete++;
                    autoShootTimer.reset();
                    autoShootState = 4;
                }
                break;

            case 4: // Kicker retract, then advance
                if (autoShootTimer.seconds() >= SpindexConfig.kickRetractDelay) {
                    if (shotsComplete >= 3 || !hasLoadedChamber()) {
                        autoShootTimer.reset();
                        autoShootState = 13; // finish
                    } else {
                        advanceToNextLoadedChamber();
                        autoShootTimer.reset();
                        autoShootState = 5;
                    }
                }
                break;

            // ======================= BALL 2 =======================
            case 5: // Wait for sorter to settle
                if (autoShootTimer.seconds() >= SpindexConfig.sorterSettleTime
                        && atTarget()) {
                    autoShootTimer.reset();
                    autoShootState = 6;
                }
                break;

            case 6: // Wait for flywheel RPM
                if (outtake.getRPM() >= Outtake.OuttakeConfig.closeRPM
                        - SpindexConfig.rpmThresholdOffset) {
                    kicker.up();
                    autoShootTimer.reset();
                    autoShootState = 7;
                }
                break;

            case 7: // Ball in flight
                if (autoShootTimer.seconds() >= SpindexConfig.kickUpDuration) {
                    kicker.down();
                    chamberColors[currentChamber] = 'E';
                    slotStatus[currentChamber]    = false;
                    shotsComplete++;
                    autoShootTimer.reset();
                    autoShootState = 8;
                }
                break;

            case 8: // Kicker retract, then advance
                if (autoShootTimer.seconds() >= SpindexConfig.kickRetractDelay) {
                    if (shotsComplete >= 3 || !hasLoadedChamber()) {
                        autoShootTimer.reset();
                        autoShootState = 13;
                    } else {
                        advanceToNextLoadedChamber();
                        autoShootTimer.reset();
                        autoShootState = 9;
                    }
                }
                break;

            // ======================= BALL 3 =======================
            case 9: // Wait for sorter to settle
                if (autoShootTimer.seconds() >= SpindexConfig.sorterSettleTime
                        && atTarget()) {
                    autoShootTimer.reset();
                    autoShootState = 10;
                }
                break;

            case 10: // Wait for flywheel RPM
                if (outtake.getRPM() >= Outtake.OuttakeConfig.closeRPM
                        - SpindexConfig.rpmThresholdOffset) {
                    kicker.up();
                    autoShootTimer.reset();
                    autoShootState = 11;
                }
                break;

            case 11: // Ball in flight
                if (autoShootTimer.seconds() >= SpindexConfig.kickUpDuration) {
                    kicker.down();
                    chamberColors[currentChamber] = 'E';
                    slotStatus[currentChamber]    = false;
                    shotsComplete++;
                    autoShootTimer.reset();
                    autoShootState = 12;
                }
                break;

            case 12: // Kicker retract before finish
                if (autoShootTimer.seconds() >= SpindexConfig.kickRetractDelay) {
                    autoShootTimer.reset();
                    autoShootState = 13;
                }
                break;

            // ======================= FINISH =======================
            case 13:
                if (autoShootTimer.seconds() >= SpindexConfig.modeToggleWaitTime) {
                    state       = SpindexState.INTAKE_STATIC;
                    outtakeMode = false;
                    autoShootState = 0;
                    shotsComplete  = 0;

                    targetPos = intakePosition(currentChamber);

                    if (gamepad != null) {
                        gamepad.rumble(500);
                    }
                }
                break;
        }
    }

    private boolean hasLoadedChamber() {
        for (int i = 0; i < 3; i++) {
            if (chamberColors[i] != 'E') return true;
        }
        return false;
    }

    public boolean allChambersFull() {
        return chamberColors[0] != 'E'
            && chamberColors[1] != 'E'
            && chamberColors[2] != 'E';
    }



    public void addIndex() {
        if (state == SpindexState.SHOOTING) return;
        currentChamber = Math.floorMod(currentChamber + 1, 3);
        state     = SpindexState.SWITCHING_CHAMBERS;
        targetPos = positionForChamber(currentChamber);
    }


    public void subtractIndex() {
        if (state == SpindexState.SHOOTING) return;
        currentChamber = Math.floorMod(currentChamber - 1, 3);
        state     = SpindexState.SWITCHING_CHAMBERS;
        targetPos = positionForChamber(currentChamber);
    }

    public void setIndex(int i) {
        currentChamber = Math.floorMod(i, 3);
        targetPos = positionForChamber(currentChamber);
    }

    public int getIndex() {
        return currentChamber;
    }


    public void setMode(boolean outtake) {
        if (state == SpindexState.SHOOTING) return;
        outtakeMode = outtake;
        targetPos   = positionForChamber(currentChamber);
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


    public void moveToPos(double target, boolean absEncoder) {
        updatePController();
        if (state == SpindexState.SHOOTING) {
            updateAutoShootSequence(null);
        }
    }

    public void moveToPos(double target, int mode) {
        updatePController();
        if (state == SpindexState.SHOOTING) {
            updateAutoShootSequence(null);
        }
    }


    public void autoLoad(ColorFetch colorSensor) {
        if (state == SpindexState.SHOOTING) return;
        if (!outtakeMode && state == SpindexState.INTAKE_STATIC && autoLoadMode) {
            autoIntakeColorCheck();
        }
    }


    public void autoLaunch(KickerSpindex kicker) {
        if (!autoLaunchMode || slotStatus[currentChamber]) {
            ballFound = false;
            return;
        }
        if (!ballFound) {
            autoLaunchTimer.reset();
            ballFound = true;
        }
        if (autoLaunchTimer.milliseconds() >= SpindexConfig.launchTime
                && Math.abs(spindexMotor.getPower()) < 0.01) {
            for (int i = 0; i < 3; i++) {
                int idx = (currentChamber + i + 1) % 3;
                if (slotStatus[idx]) {
                    currentChamber = idx;
                    targetPos = positionForChamber(currentChamber);
                    state = SpindexState.SWITCHING_CHAMBERS;
                    ballFound = false;
                    break;
                }
            }
        }
    }


    public boolean[] getSlotStatus() {
        return slotStatus;
    }

    public char[] getSlotColorStatus() {
        return chamberColors;
    }

    public void addBall(int index) {
        slotStatus[index] = true;
    }

    public void clearBall(int index) {
        slotStatus[index]    = false;
        chamberColors[index] = 'E';
        ballLatched = false;
    }

    public void setSlotColorStatus(char color) {
        if (chamberColors[currentChamber] == 'E') {
            chamberColors[currentChamber] = color;
        }
    }

    public void clearSlot(int index) {
        chamberColors[index] = 'E';
    }


    public double getPos() {
        return currentPos;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return spindexMotor.getPower();
    }

    public double getAmps() {
        return spindexMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getVoltage() {
        return spindexPos.getVoltage();
    }

    public boolean atTarget() {
        return Math.abs(error) <= SpindexConfig.tolerance;
    }

    public boolean atTarget(int tolerance) {
        return Math.abs(error) <= tolerance;
    }

    public boolean isBusy() {
        return state == SpindexState.SWITCHING_CHAMBERS
            || state == SpindexState.SHOOTING;
    }

    public SpindexState getState() {
        return state;
    }

    public Motif getMotif() {
        return currentMotif;
    }

    public void setPower(int power) {
        spindexMotor.setPower(power);
    }


    public void initAbsAndRel() { }

    public void storeThreadLoopTime(double milliseconds) {
        threadLoopTime = milliseconds;
    }

    public double getThreadLoopTime() {
        return threadLoopTime;
    }

    public void exitProgram() {
        terminate = true;
    }

    public boolean getProgramState() {
        return terminate;
    }

    // ========================================================================
    // TELEMETRY
    // ========================================================================

    public void postTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== Spindex ===");
        telemetry.addData("State",        state);
        telemetry.addData("Chamber",      currentChamber);
        telemetry.addData("Outtaking?",   outtakeMode);
        telemetry.addData("Pos (deg)",    "%.1f", currentPos);
        telemetry.addData("Target (deg)", "%.1f", targetPos);
        telemetry.addData("Error",        "%.1f", error);
        telemetry.addData("At Target?",   atTarget());
        telemetry.addData("Motor Power",  "%.2f", spindexMotor.getPower());

        telemetry.addLine("--- Chambers ---");
        String ch0 = chamberColors[0] != 'E' ? String.valueOf(chamberColors[0]) : "X";
        String ch1 = chamberColors[1] != 'E' ? String.valueOf(chamberColors[1]) : "X";
        String ch2 = chamberColors[2] != 'E' ? String.valueOf(chamberColors[2]) : "X";
        telemetry.addData("Ch 0/1/2", ch0 + " / " + ch1 + " / " + ch2);
        telemetry.addData("Slots",
                slotStatus[0] + " " + slotStatus[1] + " " + slotStatus[2]);
        telemetry.addData("All Full?", allChambersFull());

        telemetry.addLine("--- Motif ---");
        telemetry.addData("Motif", currentMotif);

        telemetry.addLine("--- Auto Shoot ---");
        telemetry.addData("Shoot State",    autoShootState);
        telemetry.addData("Shots Complete",  shotsComplete);
        telemetry.addData("Auto Load",      autoLoadMode);
        telemetry.addLine();
    }
}
