package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class UpdateSpindex extends Thread{
    Spindex spindex = null;

    public UpdateSpindex(Spindex spindex){
        this.spindex = spindex;
    }

    public void run(){
        ElapsedTime loopTime = new ElapsedTime();
        while (!spindex.getProgramState()){
            loopTime.reset();
            if (spindex.isOuttakeing()){
                spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], 3);
            }
            else{
                spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], 3);
            }
            spindex.storeThreadLoopTime(loopTime.milliseconds());
        }
    }
}