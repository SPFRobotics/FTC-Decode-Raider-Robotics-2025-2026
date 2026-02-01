package org.firstinspires.ftc.teamcode.Game.Auto.Spindex;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

@Autonomous(name="Auto Far Red")
public class AutoRedFarSpindex extends LinearOpMode {
    Spindex spindex = null;
    ElapsedTime timer = new ElapsedTime();
    MecanumChassis chassis = null;

    public void moveSpindex(boolean outtaking) {
        if (outtaking) {
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], true);
        } else {
            spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], true);
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
        ColorFetch colorSensor = new ColorFetch(hardwareMap);

        chassis.initializeMovement();
        spindex.setAutoLoadMode(true);
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();

        waitForStart();

        outtake.setRPM(Outtake.OuttakeConfig.farRPM-50);
        intake.setPower(1);
        int step = 0;
        int cycles = 0;
        int rows = 0;
        chassis.run_using_encoders_all();

        // Initial rotate -20 degrees (flipped from blue)
        chassis.rotate(-20, 0.2);
        spindex.setMode(true);
        timer.reset();

        while (opModeIsActive()) {
            telemetry.addData("Color Sensor Distance", colorSensor.getDistance());
            telemetry.addData("list", spindex.getSlotStatus()[0]+ " "+spindex.getSlotStatus()[1]+" "+ spindex.getSlotStatus()[2]);
            telemetry.update();

            switch (step) {
                case 0: // Wait for RPM
                    if (outtake.getRPM() >= Outtake.OuttakeConfig.farRPM-50) {
                        step++;
                        timer.reset();
                    }
                    break;
                case 1: // Kicker up (shoot)
                    kicker.up();
                    if (timer.seconds() >= 0.3) {
                        cycles++;
                        step++;
                        timer.reset();
                        spindex.clearBall(spindex.getIndex());
                    }
                    break;
                case 2: // Kicker down
                    kicker.down();
                    if (timer.seconds() >= 0.3) {
                        step++;
                        timer.reset();
                    }
                    break;
                case 3: // Check if done shooting first 3
                    if (cycles == 3) {
                        step = 5; // Move to intake sequence
                        break;
                    }
                    spindex.addIndex();
                    step++;
                    timer.reset();
                    break;
                case 4: // Wait before next shot
                    if (timer.seconds() >= 1) {
                        step = 0;
                        timer.reset();
                    }
                    break;

                case 5: // Rotate back +20 (flipped from blue)
                    chassis.rotate(20, 0.8);
                    step++;
                    break;
                case 6: // Drive forward 20 inches
                    chassis.moveWLoop(0.8, 'f', 20);
                    spindex.setMode(false);
                    step++;
                    break;
                case 7: // Wait for forward move complete
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        step++;
                    }
                    break;
                case 8: // Rotate -90 degrees (flipped from blue)
                    chassis.rotate(-90, 0.8);
                    step++;
                    break;


                case 9: // Start intake and move forward 37 inches
                    intake.setPower(1);
                    chassis.move(0.8,"forward",12);
                    chassis.moveWLoop(0.05, 'f', 34-12);
                    step++;
                    break;
                case 10: // Intake while moving (autoLoad)
                    spindex.autoLoad(colorSensor);
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        spindex.setMode(true);
                        step++;
                    }
                    break;


                case 11: // Drive back 37 inches
                    chassis.moveWLoop(0.8, 'b', 30);
                    step++;
                    break;
                case 12: // Wait for back move complete
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        step++;
                    }
                    break;
                case 13: // Rotate back +90 (flipped from blue)
                    chassis.rotate(90, 0.8);
                    step++;
                    break;
                case 14: // Drive back 20 inches
                    chassis.moveWLoop(0.8, 'b', 20);
                    step++;
                    break;
                case 15: // Wait for back move complete
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        step++;
                    }
                    break;
                case 16: // Rotate -23 degrees for shooting (flipped from blue)
                    chassis.rotate(-23, 0.8);
                    cycles = 0;
                    rows++;
                    step++;
                    timer.reset();
                    break;


                case 17: // Wait for RPM
                    if (outtake.getRPM() >= Outtake.OuttakeConfig.farRPM-50) {
                        step++;
                        timer.reset();
                    }
                    break;
                case 18: // Kicker up (shoot)
                    kicker.up();
                    if (timer.seconds() >= 0.3) {
                        cycles++;
                        step++;
                        timer.reset();
                        spindex.clearBall(spindex.getIndex());
                    }
                    break;
                case 19: // Kicker down
                    kicker.down();
                    if (timer.seconds() >= 0.3) {
                        step++;
                        timer.reset();
                    }
                    break;
                case 20: // Check if done shooting second 3
                    if (cycles == 3) {
                        step = 22; // Done
                        break;
                    }
                    spindex.addIndex();
                    step++;
                    timer.reset();
                    break;
                case 21: // Wait before next shot
                    if (timer.seconds() >= 1) {
                        step = 17;
                        timer.reset();
                    }
                    break;

                case 22: // Drive forward 12 inches for leave points
                    chassis.moveWLoop(0.8, 'f', 12);
                    step++;
                    break;
                case 23: // Wait for move complete
                    if (!chassis.motorsAreBusy()) {
                        chassis.powerZero();
                        step++;
                    }
                    break;
                case 24: // End
                    outtake.setRPM(0);
                    intake.setPower(0);
                    requestOpModeStop();
                    break;
            }
            moveSpindex(spindex.isOuttakeing());
            led.cycleColors(10);

            telemetry.addData("Step", step);
            telemetry.addData("Cycles", cycles);
            telemetry.addData("Rows", rows);
            telemetry.update();
        }
    }
}
