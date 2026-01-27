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

@Autonomous(name="Auto Short Blue")
public class AutoShortBlueSpindex extends LinearOpMode {
    Spindex spindex = null;
    ElapsedTime timer = new ElapsedTime();
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
        chassis.initializeMovement();
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();
        waitForStart();

        outtake.setRPM(Outtake.OuttakeConfig.closeRPM);
        int step = 0;
        int cycles = 0;
        chassis.run_using_encoders_all();

        //Move back 48 inches
        intake.setPower(1);
        chassis.moveWLoop(0.6, "backward", 30);

        //Move spindex to outtake position
        while (opModeIsActive() && chassis.motorsAreBusy()){
            moveSpindex(true);
            led.cycleColors(10);

            telemetry.addData("Back Left: ", chassis.backLeft.getCurrentPosition());
            telemetry.addData("Back Right: ", chassis.backRight.getCurrentPosition());
            telemetry.addData("Front Right:", chassis.frontRight.getCurrentPosition());
            telemetry.addData("Front Left", chassis.frontLeft.getCurrentPosition());
            telemetry.update();
        }
        chassis.powerZero();
        timer.reset();
        while (opModeIsActive()) {
            led.cycleColors(10);
            switch (step) {
                case 0:
                    if (outtake.getRPM() >= Outtake.OuttakeConfig.closeRPM){
                        step++;
                        timer.reset();
                    }
                    break;
                case 1:
                    kicker.up();
                    if (timer.seconds() >= 0.2) {
                        cycles++;
                        step++;
                        timer.reset();
                    }
                    break;
                case 2:
                    kicker.down();
                    if (timer.seconds() >= 0.2) {
                        step++;
                        timer.reset();
                    }
                    break;
                case 3:
                    if (cycles == 3){
                        step = 5;
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

        chassis.move(1, "left", 24);

        outtake.setRPM(0);
        intake.setPower(0);
        kicker.down();
        spindex.setPower(0);
        while (opModeIsActive()){
            led.cycleColors(10);
        }
    }
}
