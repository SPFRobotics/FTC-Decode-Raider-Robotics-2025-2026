package org.firstinspires.ftc.teamcode.Game.Auto.PedroPaths.TestPaths;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Subsystems.ColorFinder;

import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.intakePos;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.outtakePos;
@Disabled
//@Autonomous(name = "Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.", group = "Autonomous")
public class SpindexTestPath extends OpMode {

    private static final double SHOOT_RPM = Outtake.OuttakeConfig.closeRPM;
    private static final int INTAKING = 0;
    private static final int SHOOTING = 1;
    private static final int ADVANCING = 2;
    private static final int DONE = 3;
    private static final double INTAKE_DISTANCE_CM = 5.7;
    private static final double RELEASE_DISTANCE_CM = 7;
    private static final char COLOR_P = 'P';
    private static final char COLOR_G = 'G';
    private static final char COLOR_U = 'U';
    private static final char COLOR_E = 'E';

    private TelemetryManager panelsTelemetry;
    private Outtake outtake;
    private Spindex spindex;
    private Intake intake;

    private KickerSpindex kicker;
    private ColorFinder colorSensor;
    private Limelight limelight;
    private int state = DONE;
    private int shotIndex = 0;      // which slot we are on (0,1,2)
    private int lastCycleCount = 0; // last observed kicker cycle
    private int ballCount = 0;
    private boolean ballLatched = false;
    private final char[] slotColors = {COLOR_E, COLOR_E, COLOR_E};
    private final int[] shootOrder = {0, 1, 2};
    private boolean usingFallback = false;
    private String fallbackReason = "not set";
    private int motifId = -1;
    private int lastDetectedMotif = -1;
    private String lastFidList = "";
    private boolean llValid = false;
    private int fidCount = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        outtake = new Outtake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        intake = new Intake(hardwareMap);
        colorSensor = new ColorFinder(hardwareMap);
        limelight = new Limelight(hardwareMap);
        limelight.start();

        outtake.resetKickerCycle();
        outtake.setRPM(SHOOT_RPM);
        intake.intakeOn();
        ballCount = 0;
        shotIndex = 0;
        lastCycleCount = 0;
        spindex.setIndex(shotIndex);
        state = INTAKING;

        panelsTelemetry.debug("Status", "Shooting 3 via spindex (no drive)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        int detected = detectMotifInline();
        if (detected != -1) {
            lastDetectedMotif = detected;
        }
        motifId = lastDetectedMotif;
        int currentCycles = outtake.getKickerCycleCount();
        intake.intakeOn();

        // Early exit when all shots are done
        if (state == DONE || shotIndex >= 3) {
            outtake.setRPM(0);
            intake.intakeOff();
            state = DONE;
            requestOpModeStop();
            return;
        }

        switch (state) {
            case INTAKING:
                spindex.setIndex(ballCount % 3);
                double intakeTarget = intakePos[spindex.getIndex()];
                spindex.moveToPos(intakeTarget, true);

                double distance = colorSensor != null ? colorSensor.getDistance() : Double.POSITIVE_INFINITY;
                if (distance <= INTAKE_DISTANCE_CM && !ballLatched && ballCount < 3 && spindex.getPower() == 0) {
                    int slot = spindex.getIndex();
                    slotColors[slot] = detectColor();
                    ballLatched = true;
                    ballCount++;
                    spindex.addIndex();
                } else if (distance > RELEASE_DISTANCE_CM) {
                    ballLatched = false;
                }

                if (ballCount >= 3) {
                    //intake.intakeOff();
                    outtake.resetKickerCycle();
                    outtake.setRPM(SHOOT_RPM);
                    determineShootOrder();
                    shotIndex = 0;
                    lastCycleCount = currentCycles;
                    spindex.setIndex(shootOrder[shotIndex]);
                    state = SHOOTING;
                }
                break;

            case SHOOTING:
                spindex.setIndex(shootOrder[shotIndex]);
                double targetAngle = outtakePos[spindex.getIndex()];
                spindex.moveToPos(targetAngle, true);
                double error = Math.abs(AngleUnit.normalizeDegrees(targetAngle - spindex.getPos()));
                boolean aligned = error <= Spindex.SpindexValues.tolorence;

                if (aligned) {
                    outtake.enableSpindexKickerCycle(true, SHOOT_RPM);
                }

                // When a shot completes, advance to next slot
                if (currentCycles > lastCycleCount) {
                    lastCycleCount = currentCycles;
                    shotIndex++;
                    if (shotIndex >= 3) {
                        outtake.setRPM(0);
                        kicker.down();
                        outtake.resetKickerCycle();
                        intake.intakeOff();
                        state = DONE;
                        requestOpModeStop();
                    } else {
                        state = ADVANCING;
                    }
                }
                break;

            case ADVANCING:
                // Spin to next slot before arming kicker again
                spindex.setIndex(shootOrder[shotIndex]);
                targetAngle = outtakePos[spindex.getIndex()];
                spindex.moveToPos(targetAngle, true);
                error = Math.abs(AngleUnit.normalizeDegrees(targetAngle - spindex.getPos()));
                if (error <= Spindex.SpindexValues.tolorence) {
                    state = SHOOTING;
                }
                break;

            case DONE:
            default:
                outtake.setRPM(0);
                break;
        }

        panelsTelemetry.debug("State", state);
        panelsTelemetry.debug("SpindexIndex", spindex.getIndex());
        panelsTelemetry.debug("MotifId", motifId);
        panelsTelemetry.debug("FidList", lastFidList);
        panelsTelemetry.debug("LLValid", llValid);
        panelsTelemetry.debug("FidCount", fidCount);
        panelsTelemetry.debug("SlotColors", new String(slotColors));
        panelsTelemetry.debug("Order", String.format("%d%d%d", shootOrder[0], shootOrder[1], shootOrder[2]));
        panelsTelemetry.debug("Fallback", usingFallback ? fallbackReason : "none");
        panelsTelemetry.debug("Cycles", outtake.getKickerCycleCount());
        panelsTelemetry.debug("RPM", outtake.getRPM());
        panelsTelemetry.debug("CycleTime", outtake.getCurrentCycleTime());
        panelsTelemetry.update(telemetry);
    }

    private char detectColor() {
        if (colorSensor == null) {
            return COLOR_U;
        }
        if (colorSensor.isGreen()) {
            return COLOR_G;
        }
        if (colorSensor.isPurple()) {
            return COLOR_P;
        }
        return COLOR_U;
    }

    private void determineShootOrder() {
        usingFallback = false;
        fallbackReason = "none";
        motifId = lastDetectedMotif != -1 ? lastDetectedMotif : detectMotifInline();

        char[] desiredPattern;
        if (motifId == 21) {
            desiredPattern = new char[]{COLOR_G, COLOR_P, COLOR_P};
        } else if (motifId == 22) {
            desiredPattern = new char[]{COLOR_P, COLOR_G, COLOR_P};
        } else if (motifId == 23) {
            desiredPattern = new char[]{COLOR_P, COLOR_P, COLOR_G};
        } else {
            desiredPattern = null;
        }

        boolean success = desiredPattern != null && buildOrder(desiredPattern);
        if (!success) {
            usingFallback = true;
            fallbackReason = desiredPattern == null ? "motif_invalid_or_missing" : "pattern_unachievable";
            shootOrder[0] = 0;
            shootOrder[1] = 1;
            shootOrder[2] = 2;
        }
    }

    private int detectMotifInline() {
        if (limelight == null) return -1;
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            llValid = false;
            fidCount = 0;
            lastFidList = "null";
            return -1;
        }
        llValid = result.isValid();
        java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            fidCount = 0;
            lastFidList = "";
            return -1;
        }
        fidCount = fiducials.size();
        StringBuilder sb = new StringBuilder();
        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (sb.length() > 0) sb.append(",");
            sb.append(id);
            if (id == 21 || id == 22 || id == 23) {
                lastFidList = sb.toString();
                return id;
            }
        }
        lastFidList = sb.toString();
        return -1;
    }

    private boolean buildOrder(char[] desired) {
        boolean[] used = new boolean[3];
        for (int i = 0; i < desired.length; i++) {
            int idx = findSlot(desired[i], used);
            if (idx == -1) {
                return false;
            }
            shootOrder[i] = idx;
            used[idx] = true;
        }
        return true;
    }

    private int findSlot(char desiredColor, boolean[] used) {
        for (int i = 0; i < slotColors.length; i++) {
            if (!used[i] && slotColors[i] == desiredColor) {
                return i;
            }
        }
        return -1;
    }
}

