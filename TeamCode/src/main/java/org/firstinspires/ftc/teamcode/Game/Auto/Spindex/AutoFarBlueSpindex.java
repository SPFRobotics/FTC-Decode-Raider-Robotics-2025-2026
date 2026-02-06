package org.firstinspires.ftc.teamcode.Game.Auto.Spindex;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

@Autonomous(name="Auto Far Blue ")
public class AutoFarBlueSpindex extends LinearOpMode {

    Spindex spindex = null;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime spindexSettle = new ElapsedTime();
    MecanumChassis chassis = null;

    private void moveSpindex(boolean outtaking) {
        if (outtaking) {
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], true);
        } else {
            spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], true);
        }
    }

    private void spindexUpdateSafe() {
        if (!chassis.motorsAreBusy()) {
            moveSpindex(spindex.isOuttakeing());
        } else {
            spindex.setPower(0);
        }
    }

    private boolean spindexReadyForShot() {
        moveSpindex(spindex.isOuttakeing());
        return spindex.atTarget();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new MecanumChassis(this);
        spindex = new Spindex(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap, true);
        Intake intake = new Intake(hardwareMap);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        LedLights led = new LedLights(hardwareMap);
        ColorFetch colorSensor = new ColorFetch(hardwareMap);

        chassis.initializeMovement();
        spindex.setAutoLoadMode(true);

        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();

        waitForStart();

        outtake.setRPM(Outtake.OuttakeConfig.farRPM - 50);
        intake.setPower(1);

        int step = 0;
        int cycles = 0;
        int rows = 0;

        chassis.run_using_encoders_all();

        chassis.rotate(20, 0.2);
        spindex.setMode(true);
        spindexSettle.reset();

        while (opModeIsActive()) {

            telemetry.addData("Color Sensor Distance", colorSensor.getDistance());
            telemetry.addData("list", spindex.getSlotStatus()[0] + " " + spindex.getSlotStatus()[1] + " " + spindex.getSlotStatus()[2]);
            telemetry.addData("Step", step);
            telemetry.addData("Cycles", cycles);
            telemetry.addData("Rows", rows);
            telemetry.addData("SpindexErr", spindex.getError());
            telemetry.addData("SpindexPwr", spindex.getPower());
            telemetry.addData("AtTarget", spindex.atTarget());
            telemetry.update();

            led.cycleColors(10);

            if (spindexSettle.seconds() < 0.35) {
                moveSpindex(spindex.isOuttakeing());
            } else {
                spindexUpdateSafe();
            }

            switch (step) {

                case 0:
                    if (outtake.getRPM() >= Outtake.OuttakeConfig.farRPM - 50 && spindexReadyForShot()) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 1:
                    kicker.up();
                    if (timer.seconds() >= 0.30) {
                        cycles++;
                        spindex.clearBall(spindex.getIndex());
                        step++;
                        timer.reset();
                    }
                    break;

                case 2:
                    kicker.down();
                    if (timer.seconds() >= 0.30) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 3:
                    if (cycles >= 3) {
                        step = 5;
                        break;
                    }
                    spindex.addIndex();
                    spindexSettle.reset();
                    step = 0;
                    break;

                case 5:
                    chassis.rotate(-20, 0.8);
                    step++;
                    break;

                case 6:
                    chassis.moveWLoop(0.8, 'f', 20);
                    spindex.setMode(false);
                    spindexSettle.reset();
                    step++;
                    break;

                case 7:
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        step++;
                    }
                    break;

                case 8:
                    chassis.rotate(90, 0.8);
                    step++;
                    break;

                case 9:
                    intake.setPower(1);
                    chassis.move(0.8, "forward", 12);
                    chassis.moveWLoop(0.05, 'f', 34 - 12);
                    step++;
                    break;

                case 10:
                    spindex.autoLoad(colorSensor);
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        spindex.setMode(true);
                        spindexSettle.reset();
                        step++;
                    }
                    break;

                case 11:
                    chassis.moveWLoop(0.8, 'b', 30);
                    step++;
                    break;

                case 12:
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        step++;
                    }
                    break;

                case 13:
                    chassis.rotate(-90, 0.8);
                    step++;
                    break;

                case 14:
                    chassis.moveWLoop(0.8, 'b', 20);
                    step++;
                    break;

                case 15:
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        step++;
                    }
                    break;

                case 16:
                    chassis.rotate(23, 0.8);
                    cycles = 0;
                    rows++;
                    spindexSettle.reset();
                    step++;
                    timer.reset();
                    break;

                case 17:
                    if (outtake.getRPM() >= Outtake.OuttakeConfig.farRPM - 50 && spindexReadyForShot()) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 18:
                    kicker.up();
                    if (timer.seconds() >= 0.30) {
                        cycles++;
                        spindex.clearBall(spindex.getIndex());
                        step++;
                        timer.reset();
                    }
                    break;

                case 19:
                    kicker.down();
                    if (timer.seconds() >= 0.30) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 20:
                    if (cycles >= 3) {
                        step = 22;
                        break;
                    }
                    spindex.addIndex();
                    spindexSettle.reset();
                    step = 17;
                    break;

                case 22:
                    chassis.moveWLoop(0.8, 'f', 12);
                    step++;
                    break;

                case 23:
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        step++;
                    }
                    break;

                case 24:
                    outtake.setRPM(0);
                    intake.setPower(0);
                    requestOpModeStop();
                    break;
            }
        }
    }
}
