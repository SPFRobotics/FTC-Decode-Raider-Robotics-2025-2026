package org.firstinspires.ftc.teamcode.Game.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassisMod;

@Autonomous(name="Zucking It")
public class ZuckingIt extends LinearOpMode {
    Spindex spindex = null;
    ElapsedTime time = new ElapsedTime();
    public void moveSpindex(){
        while (opModeIsActive()){
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], true);
            if (spindex.getPower() == 0){
                break;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumChassis chassis = new MecanumChassis(this);
        spindex = new Spindex(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap, true);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        chassis.initializeMovement();
        waitForStart();
        //send move command
        while (opModeIsActive() && chassis.motorsAreBusy()){
           if (!chassis.moveWLoop(1, "backward", 48)){
               chassis.powerZero();
               break;
           }
        }
    }
}
