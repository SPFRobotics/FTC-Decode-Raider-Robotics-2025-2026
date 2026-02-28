package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Spindex.SpindexValues.pidf;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PassiveSpindex extends Spindex{
    boolean cycling = false;
    int greenPatternIndex = 0;

    public PassiveSpindex(HardwareMap hardwareMap, char[] pattern) {
        super(hardwareMap);
        for (int i = 0; i < 3; i++){
            if (pattern[i] == 'G'){
                greenPatternIndex = i;
                break;
            }
        }
    }

    public void sort(){
        super.setMode(false);
        super.setIndex(super.getIndexOfColor('G'));
        int difference = greenPatternIndex - super.getIndexOfColor('G');
        for (int i = 0; i < Math.abs(difference); i++){
            if (difference < 0){
                super.subtractIndex();
            }
            else if (difference > 0){
                super.addIndex();
            }
        }
    }

    public void setCycling(boolean x){
        cycling = x;
    }

    public boolean isCycling(){
        return cycling;
    }


    public void moveTicks(int target, int mode) {
        spindexMotor.setTargetPosition(target);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setPower(1);
    }
}
