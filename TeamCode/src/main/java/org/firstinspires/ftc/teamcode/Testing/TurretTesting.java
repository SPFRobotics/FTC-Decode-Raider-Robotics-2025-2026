package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.NextOuttake;

@Disabled
public class TurretTesting extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        NextOuttake outtake = NextOuttake.INSTANCE;
        outtake.initialize();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                outtake.setRPM(NextOuttake.closeRPM);
            }
            else if (gamepad1.y){
                outtake.setRPM(NextOuttake.farRPM);
            }
            else{
                outtake.setRPM(0);
            }
        }
    }

}
