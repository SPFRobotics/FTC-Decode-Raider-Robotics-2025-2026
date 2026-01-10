package org.firstinspires.ftc.teamcode.Game.Subsystems;

public class UpdateSpindex extends Thread{
    Spindex spindex = null;

    public UpdateSpindex(Spindex spindex){
        this.spindex = spindex;
    }

    public void run(){
        for(;;){
            if (spindex.getMode()){
                spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], true);
            }
            else{
                spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], true);
            }
        }
    }
}
