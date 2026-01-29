package org.firstinspires.ftc.teamcode.Game.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

@Autonomous(name="Auto Far Blue")
public class AutoFarBlue extends LinearOpMode {

    Spindex spindex = null;
    ElapsedTime timer = new ElapsedTime();
    MecanumChassis chassis = null;

    private static final double INTAKE_RUN_SEC = 1.4;
    private static final double SPINDEX_ADVANCE_EVERY_SEC = 0.45;

    private static final double DRIVE_PWR = 0.5;
    private static final double INTAKE_PWR = 0.6;

    public void moveSpindex(boolean outtaking) {
        if (outtaking) {
            spindex.moveToPos(
                    Spindex.SpindexValues.outtakePos[spindex.getIndex()], true
            );
        } else {
            spindex.moveToPos(
                    Spindex.SpindexValues.intakePos[spindex.getIndex()], true
            );
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        chassis = new MecanumChassis(this);
        spindex = new Spindex(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap, true);
        Intake intake = new Intake(hardwareMap);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        LedLights led = new LedLights(hardwareMap);

        chassis.initializeMovement();
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();

        waitForStart();
        if (!opModeIsActive()) return;

        outtake.setRPM(Outtake.OuttakeConfig.farRPM);

        int step = 0;
        int cycles = 0;

        chassis.run_using_encoders_all();

        chassis.rotate(20, DRIVE_PWR);

        timer.reset();
        while (opModeIsActive()) {
            led.cycleColors(10);

            switch (step) {
                case 0:
                    if (outtake.getRPM() >= Outtake.OuttakeConfig.farRPM) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 1:
                    kicker.up();
                    if (timer.seconds() >= 0.5) {
                        cycles++;
                        step++;
                        timer.reset();
                    }
                    break;

                case 2:
                    kicker.down();
                    if (timer.seconds() >= 0.3) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 3:
                    if (cycles == 3) {
                        step = 5;
                        break;
                    }
                    spindex.addIndex();
                    step++;
                    timer.reset();
                    break;

                case 4:
                    if (timer.seconds() >= 0.5) {
                        step = 0;
                        timer.reset();
                    }
                    break;
            }

            moveSpindex(true);

            if (cycles == 3) {
                break;
            }
        }

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 0.3) {
            kicker.down();
            led.cycleColors(10);
        }

        chassis.rotate(-20, DRIVE_PWR);

        chassis.moveWLoop(DRIVE_PWR, 'f', 20);
        while (opModeIsActive() && chassis.motorsAreBusy()) {
            moveSpindex(false);
            led.cycleColors(10);
        }
        chassis.powerZero();

        chassis.rotate(90, DRIVE_PWR);

        double INTAKE_DRIVE_PWR = 0.25;   // MUCH slower into balls
        double INTAKE_TOTAL_TIME = 2.6;  // longer so intake actually works

        intake.setPower(INTAKE_PWR);
        chassis.moveWLoop(INTAKE_DRIVE_PWR, 'f', 33);

        timer.reset();
        int advances = 0;

        while (opModeIsActive()) {
            led.cycleColors(10);
            moveSpindex(false);

            double targetTime = SPINDEX_ADVANCE_EVERY_SEC * (advances + 1);
            if (advances < 3 && timer.seconds() >= targetTime) {
                spindex.addIndex();
                advances++;
            }

            // let it keep driving UNTIL time is up
            if (timer.seconds() >= INTAKE_TOTAL_TIME) {
                break;
            }

            telemetry.addData("IntakeTime", timer.seconds());
            telemetry.addData("Advances", advances);
            telemetry.update();
        }

        chassis.powerZero();
    }
}