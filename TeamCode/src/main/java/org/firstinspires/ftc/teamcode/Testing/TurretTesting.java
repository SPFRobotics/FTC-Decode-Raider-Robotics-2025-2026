package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;

@TeleOp
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
