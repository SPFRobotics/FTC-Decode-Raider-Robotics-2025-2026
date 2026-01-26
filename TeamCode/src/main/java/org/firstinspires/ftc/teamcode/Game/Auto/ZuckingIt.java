package org.firstinspires.ftc.teamcode.Game.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassisMod;

@Autonomous(name="Zucking It")
public class ZuckingIt extends LinearOpMode {
    Spindex spindex = null;
    ElapsedTime timer = new ElapsedTime();
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
        MecanumChassis chassis = new MecanumChassis(this);
        spindex = new Spindex(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap, true);
        Intake intake = new Intake(hardwareMap);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        LedLights led = new LedLights(hardwareMap);
        chassis.initializeMovement();
        waitForStart();
        outtake.setRPM(Outtake.OuttakeConfig.closeRPM);
        int step = 0;
        int cycles = 0;

        //Move back 48 inches
        intake.setPower(1);
        chassis.moveWLoop(1, "backward", 48);

        //Move spindex to outtake position
        while (opModeIsActive() && chassis.motorsAreBusy()){
            moveSpindex(true);
            led.cycleColors(10);
        }
        chassis.powerZero();
        timer.reset();
        while (opModeIsActive()) {
            led.cycleColors(10);
            switch (step) {
                case 0:
                    kicker.up();
                    if (timer.seconds() >= 1) {
                        step++;
                        timer.reset();
                    }
                    break;
                case 1:
                    kicker.down();
                    if (timer.seconds() >= 1) {
                        step++;
                        timer.reset();
                    }
                    break;
                case 2:
                    spindex.addIndex();
                    step++;
                    timer.reset();
                    break;
                case 3:
                    if (timer.seconds() >= 1){
                        cycles++;
                        if (cycles == 3){
                            step++;
                            requestOpModeStop();
                        }
                        else{
                            step = 0;
                            timer.reset();
                        }
                    }
                    break;
            }
            moveSpindex(true);
        }
    }
}
