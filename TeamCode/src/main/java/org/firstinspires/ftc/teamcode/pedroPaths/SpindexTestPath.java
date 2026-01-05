package org.firstinspires.ftc.teamcode.pedroPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFinder;

import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.intakePos;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.outtakePos;

@Autonomous(name = "Cam UwU path", group = "Autonomous")
public class SpindexTestPath extends OpMode {

    private static final double SHOOT_RPM = Outtake.OuttakeConfig.farRPM;
    private static final int INTAKING = 0;
    private static final int SHOOTING = 1;
    private static final int ADVANCING = 2;
    private static final int DONE = 3;
    private static final double INTAKE_DISTANCE_CM = 5.0;
    private static final double RELEASE_DISTANCE_CM = 6.0;

    private Outtake outtake;
    private Spindex spindex;
    private Intake intake;

    private KickerSpindex kicker;
    private ColorFinder colorSensor;
    private int state = DONE;
    private int shotIndex = 0;      // which slot we are on (0,1,2)
    private int lastCycleCount = 0; // last observed kicker cycle
    private int ballCount = 0;
    private boolean ballLatched = false;

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        intake = new Intake(hardwareMap);
        colorSensor = new ColorFinder(hardwareMap);

        outtake.resetKickerCycle();
        outtake.setRPM(SHOOT_RPM);
        intake.intakeOn();
        ballCount = 0;
        shotIndex = 0;
        lastCycleCount = 0;
        spindex.setIndex(shotIndex);
        state = INTAKING;

        telemetry.addData("Status", "Shooting 3 via spindex (no drive)");
        telemetry.update();
    }

    @Override
    public void loop() {
        int currentCycles = outtake.getKickerCycleCount();

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
                    ballLatched = true;
                    ballCount++;
                    spindex.addIndex();
                } else if (distance > RELEASE_DISTANCE_CM) {
                    ballLatched = false;
                }

                if (ballCount >= 3) {
                    intake.intakeOff();
                    outtake.resetKickerCycle();
                    outtake.setRPM(SHOOT_RPM);
                    shotIndex = 0;
                    lastCycleCount = currentCycles;
                    spindex.setIndex(0);
                    state = SHOOTING;
                }
                break;

            case SHOOTING:
                spindex.setIndex(shotIndex);
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
                spindex.setIndex(shotIndex);
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

        telemetry.addData("State", state);
        telemetry.addData("SpindexIndex", spindex.getIndex());
        telemetry.addData("Cycles", outtake.getKickerCycleCount());
        telemetry.addData("RPM", outtake.getRPM());
        telemetry.addData("CycleTime", outtake.getCurrentCycleTime());
        telemetry.update();
    }
}

