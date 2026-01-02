package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;

@TeleOp
public class SpindexStress extends LinearOpMode {
    public void runOpMode(){
        Spindex spindex = new Spindex(hardwareMap, true);
        waitForStart();
        while (opModeIsActive()){
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()]);
            spindex.addIndex();
        }
    }
}
