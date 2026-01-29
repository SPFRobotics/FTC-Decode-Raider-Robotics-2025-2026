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

@Autonomous(name="Auto Far Blue (Time Based)")
public class AutoFarBlue extends LinearOpMode {

    Spindex spindex = null;
    ElapsedTime timer = new ElapsedTime();
    MecanumChassis chassis = null;

    // ====== TUNE THIS ======
    // How long you need driving + intaking to reliably grab 3 balls
    private static final double INTAKE_RUN_SEC = 1.4;  // start ~1.2-1.8 and tune

    // How often to advance spindex while intaking (time-based indexing)
    private static final double SPINDEX_ADVANCE_EVERY_SEC = 0.45; // tune

    public void moveSpindex(boolean outtaking){
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

        chassis.initializeMovement();
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();

        waitForStart();
        if (!opModeIsActive()) return;

        outtake.setRPM(Outtake.OuttakeConfig.farRPM);

        int step = 0;
        int cycles = 0;

        chassis.run_using_encoders_all();

        // =========================
        // ROTATE 20
        // =========================
        chassis.rotate(20, 1);

        // =========================
        // SHOOT 3 (your same time-based switch logic)
        // =========================
        timer.reset();
        while (opModeIsActive()) {
            led.cycleColors(10);

            switch (step) {
                case 0:
                    if (outtake.getRPM() >= Outtake.OuttakeConfig.farRPM){
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
                    if (cycles == 3){
                        step = 5;
                        break;
                    }
                    spindex.addIndex();
                    step++;
                    timer.reset();
                    break;

                case 4:
                    if (timer.seconds() >= 0.5){
                        step = 0;
                        timer.reset();
                    }
                    break;
            }

            moveSpindex(true);

            if (cycles == 3){
                break;
            }
        }

        // =========================
        // ROTATE -20
        // =========================
        chassis.rotate(-20, 1);

        // =========================
        // MOVE 20 FORWARD (keep it simple)
        // =========================
        chassis.moveWLoop(1, 'f', 20);
        while (opModeIsActive() && chassis.motorsAreBusy()){
            moveSpindex(false);
            led.cycleColors(10);
        }
        chassis.powerZero();

        // =========================
        // ROTATE 90 LEFT (flip sign if needed)
        // =========================
        chassis.rotate(90, 1);

        // =========================
        // MOVE 33 FORWARD + INTAKE (TIME-BASED STOP)
        // Start the 33 inch command, but we stop early once time is up.
        // =========================
        intake.setPower(1);

        chassis.moveWLoop(1, 'f', 33);

        timer.reset();
        int advances = 0;

        while (opModeIsActive() && chassis.motorsAreBusy()) {
            led.cycleColors(10);

            // Keep spindex in intake position while intaking
            moveSpindex(false);

            // Advance spindex on a cadence while collecting
            double targetTime = SPINDEX_ADVANCE_EVERY_SEC * (advances + 1);
            if (advances < 3 && timer.seconds() >= targetTime) {
                spindex.addIndex();
                advances++;
            }

            // TIME-BASED STOP: assume we collected 3 by now
            if (timer.seconds() >= INTAKE_RUN_SEC) {
                chassis.powerZero();
                break;
            }

            telemetry.addData("IntakeTime", timer.seconds());
            telemetry.addData("Advances", advances);
            telemetry.update();
        }

        chassis.powerZero();

        // =========================
        // STOP EVERYTHING
        // =========================
        outtake.setRPM(0);
        intake.setPower(0);
        kicker.down();
        spindex.setPower(0);
    }
}
