package org.firstinspires.ftc.teamcode.Game.Subsystems;

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
            if (spindex.getMode()){
                spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], true);
            }
            else{
                spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], true);
            }
            spindex.storeThreadLoopTime(loopTime.milliseconds());
        }
    }
}