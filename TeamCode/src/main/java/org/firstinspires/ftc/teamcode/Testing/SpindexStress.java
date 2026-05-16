package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextSpindex;

@Disabled
public class SpindexStress extends LinearOpMode {
    public void runOpMode(){
        NextSpindex spindex = NextSpindex.INSTANCE;
        spindex.initialize(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            //spindex.moveToPos(NextSpindex.outtakePos[spindex.getIndex()], false);
            spindex.addIndex();
        }
    }
}
