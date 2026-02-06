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

@Autonomous(name="Auto Short Blue")
public class AutoShortBlueSpindex extends LinearOpMode {
    Spindex spindex = null;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    MecanumChassis chassis = null;
    public void moveSpindex(boolean outtaking){
        if (outtaking) {
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], true);
        }
        else{
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
        runtime.reset();

        intake.setPower(1);
        outtake.setRPM(Outtake.OuttakeConfig.closeRPM);
        int step = 0;
        int cycles = 0;
        int rows = 0;
        chassis.run_using_encoders_all();

        //Move back 48 inches
        //intake.setPower(1);
        chassis.moveWLoop(0.8, 'b', 48);
        spindex.setMode(true);

        //Move spindex to outtake position
        while (opModeIsActive() && chassis.motorsAreBusy()){
            led.cycleColors(10);
            moveSpindex(spindex.isOuttakeing());
        }
        chassis.powerZero();
        timer.reset();
        while (opModeIsActive()) {
            switch (step) {
                case 0:
                    if (outtake.getRPM() >= Outtake.OuttakeConfig.closeRPM){
                        step++;
                        timer.reset();
                    }
                    break;
                case 1:
                    kicker.up();
                    if (timer.seconds() >= 0.3
                    ) {
                        cycles++;
                        step++;
                        timer.reset();
                        spindex.clearBall(spindex.getIndex());
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
                    if (rows == 1 && cycles == 3){
                        step = 13;
                        break;
                    }
                    if (cycles == 3){
                        step = 5;
                        break;
                    }
                    spindex.addIndex();
                    step++;
                    timer.reset();
                    break;
                case 4:
                    if (timer.seconds() >= 1){
                        step = 0;
                        timer.reset();
                    }
                    break;
                case 5:
                    chassis.rotate(45, 0.8);
                    spindex.setMode(false);
                    step++;
                    break;
                case 6:
                    chassis.moveWLoop(0.8, 'f', 10);
                    step++;
                    break;
                case 7:
                    if (!chassis.motorsAreBusy()){
                        chassis.powerZero();
                        step++;
                    }
                    break;
                case 8:
                    chassis.moveWLoop(0.08, 'f', 37-10);
                    step++;
                    break;
                case 9:
                    spindex.autoLoad(colorSensor);
                    if (!chassis.motorsAreBusy()){
                        chassis.powerZero();
                        spindex.setMode(true);
                        step++;
                    }
                    break;
                case 10:
                    chassis.moveWLoop(0.8, 'b', 35);
                    step++;
                    break;
                case 11:
                    if (!chassis.motorsAreBusy()){
                        step++;
                    }
                    break;
                case 12:
                    chassis.rotate(-45, 0.8);
                    step = 0;
                    cycles = 0;
                    rows++;
                    break;
                case 13:
                    chassis.move(0.8, 'l', 15);
                    step++;
                    break;
                case 14:
                    requestOpModeStop();
            }
            moveSpindex(spindex.isOuttakeing());
            led.cycleColors(10);
        }
    }
}