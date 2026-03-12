package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DualColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;

public class ColorSensorTest extends OpMode {
    DualColorFetch colorSensor = null;
    Spindex spindex = null;
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dash.getTelemetry();

    public void init(){
        DualColorFetch colorFetch = new DualColorFetch(hardwareMap);
        spindex = new Spindex(hardwareMap);
    }

    public void loop(){
        spindex.autoLoad(colorSensor);
        spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()]);

        telemetry.addData("Distance1", colorSensor.getDistances()[0]);
        telemetry.addData("Distance2", colorSensor.getDistances()[1]);
    }
}
