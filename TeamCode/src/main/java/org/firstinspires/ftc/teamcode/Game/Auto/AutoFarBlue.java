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

@Autonomous(name="Auto Far Blue (ShortLogic)")
public class AutoFarBlue extends LinearOpMode {

    Spindex spindex;
    MecanumChassis chassis;
    ElapsedTime timer = new ElapsedTime();

    private static final double DRIVE_PWR = 0.5;
    private static final double INTAKE_PWR = 0.55;

    @Override
    public void runOpMode() throws InterruptedException {

        chassis = new MecanumChassis(this);
        spindex = new Spindex(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap, true);
        Intake intake = new Intake(hardwareMap);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        LedLights led = new LedLights(hardwareMap);

        chassis.initializeMovement();
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        waitForStart();
        if (!opModeIsActive()) return;

        outtake.setRPM(Outtake.OuttakeConfig.farRPM);
        chassis.run_using_encoders_all();

        int step = 0;
        int shots = 0;
        int intakeIndexes = 0;

        timer.reset();

        while (opModeIsActive()) {

            led.cycleColors(10);

            switch (step) {

                case 0: // rotate 20
                    chassis.rotate(20, DRIVE_PWR);
                    step++;
                    timer.reset();
                    break;

                case 1: // wait for RPM
                    if (outtake.getRPM() >= Outtake.OuttakeConfig.farRPM) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 2: // kicker up (shoot)
                    kicker.up();
                    if (timer.seconds() >= 0.5) {
                        shots++;
                        step++;
                        timer.reset();
                    }
                    break;

                case 3: // kicker down
                    kicker.down();
                    if (timer.seconds() >= 0.3) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 4: // index spindex
                    if (shots == 3) {
                        step = 6;
                        break;
                    }
                    spindex.addIndex();
                    step++;
                    timer.reset();
                    break;

                case 5: // pause before next shot
                    if (timer.seconds() >= 0.5) {
                        step = 2;
                        timer.reset();
                    }
                    break;

                case 6: // force kicker down
                    kicker.down();
                    if (timer.seconds() >= 0.35) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 7: // rotate back -20
                    chassis.rotate(-20, DRIVE_PWR);
                    step++;
                    break;

                case 8: // move forward 20
                    chassis.moveWLoop(DRIVE_PWR, 'f', 20);
                    step++;
                    break;

                case 9: // wait for move complete
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        step++;
                    }
                    break;

                case 10: // rotate 90
                    chassis.rotate(90, DRIVE_PWR);
                    step++;
                    break;

                case 11: // start intake + slow creep
                    intake.setPower(INTAKE_PWR);
                    chassis.moveWLoop(0.25, 'f', 33);
                    intakeIndexes = 0;
                    timer.reset();
                    step++;
                    break;

                case 12: // intake loop
                    moveSpindex(false);

                    if (timer.seconds() >= 0.45 * (intakeIndexes + 1)
                            && intakeIndexes < 3) {
                        spindex.addIndex();
                        intakeIndexes++;
                    }

                    if (timer.seconds() >= 2.6) {
                        chassis.powerZero();
                        intake.setPower(0);
                        step++;
                    }
                    break;

                case 13: // end
                    outtake.setRPM(0);
                    kicker.down();
                    spindex.setPower(0);
                    break;
            }

            moveSpindex(step < 7); // outtake mode only while shooting

            telemetry.addData("Step", step);
            telemetry.addData("Shots", shots);
            telemetry.addData("IntakeIdx", intakeIndexes);
            telemetry.update();

            if (step == 13) break;
        }
    }

    private void moveSpindex(boolean outtaking) {
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
}