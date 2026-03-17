package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ZucskyLens {
    private HuskyLens huskyLens = null;

    public ZucskyLens(HardwareMap hardwareMap){
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
            telemetry.update();
            throw new RuntimeException("HuskyLens failed to initialize");
        }
    }

    public HuskyLens.Block[] getBlocks(){
        return huskyLens.blocks();
    }

    public int[] getColors(){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        int[] colors = {};
        for (int i = 0; i < blocks.length; i++){
            colors[i] = blocks[i].id;
        }
        return colors;
    }
}
