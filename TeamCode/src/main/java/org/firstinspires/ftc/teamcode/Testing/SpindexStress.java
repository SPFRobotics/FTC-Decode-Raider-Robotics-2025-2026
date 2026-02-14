package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Spindex;

@Disabled
public class SpindexStress extends LinearOpMode {
    public void runOpMode(){
        Spindex spindex = new Spindex(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], false);
            spindex.addIndex();
        }
    }
}
