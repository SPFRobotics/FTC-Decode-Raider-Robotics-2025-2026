package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.DualColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextSpindex;

public class ColorSensorTest extends OpMode {
    DualColorFetch colorSensor = null;
    NextSpindex spindex = null;
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dash.getTelemetry();

    int correct = 0;
    int runs = 0;

    public void init(){
        DualColorFetch colorFetch = new DualColorFetch(hardwareMap);
        spindex = NextSpindex.INSTANCE;
        spindex.initialize(hardwareMap);
    }

    public void loop(){
        spindex.autoLoad(colorSensor);
        spindex.moveToPos(NextSpindex.outtakePos[spindex.getIndex()]);

        if (spindex.getSlotColors()[0] != 'G' && spindex.getSlotColors()[1] != 'P' && spindex.getSlotColors()[2] != 'P'){
            runs++;
            correct++;
            for (int i = 0; i < 3; i++){
                spindex.clearBall(i);
            }
        }
        else{
            runs++;
            for (int i = 0; i < 3; i++){
                spindex.clearBall(i);
            }
        }

        telemetry.addLine("Runs: " + runs);
        telemetry.addLine("Accuracy: " + (runs != 0 ? correct/runs : runs) + "%");
    }
}
