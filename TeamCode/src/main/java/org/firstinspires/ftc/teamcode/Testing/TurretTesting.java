package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Disabled
public class TurretTesting extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Outtake outtake = new Outtake(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                outtake.setRPM(Outtake.OuttakeConfig.closeRPM);
            }
            else if (gamepad1.y){
                outtake.setRPM(Outtake.OuttakeConfig.farRPM);
            }
            else{
                outtake.setRPM(0);
            }
        }
    }

}
