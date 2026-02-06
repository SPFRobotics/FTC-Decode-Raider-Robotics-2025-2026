package org.firstinspires.ftc.teamcode.Game.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

//@Autonomous(name = "Auto Far Blue Spindex (TIME) - Preload Only", group = "Autonomous")
@Disabled
public class AutoFarBlueThreeBallTime extends LinearOpMode {

    private static final double SHOOT_RPM = 3200;

    private MecanumChassis robot;
    private Intake intake;
    private Spindex spindex;
    private Outtake outtake;

    @Override
    public void runOpMode() {

        robot = new MecanumChassis(this);
        robot.initializeMovement();

        intake = new Intake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        outtake = new Outtake(hardwareMap, true);

        waitForStart();
        if (!opModeIsActive()) return;

        spindex.setMode(false);
        spindex.setIndex(0);

        aim110();
        shoot3Timed();
        unAim110();

        outtake.setRPM(0);
        intake.intakeOff();
    }

    //METHODS
    private void aim110() {
        robot.rotate(20.0, 0.25);
    }

    private void unAim110() {
        robot.rotate(-20.0, 0.25);
    }

    private void shoot3Timed() {
        outtake.setRPM(SHOOT_RPM);
        sleep(900);

        for (int i = 0; i <= 3 && opModeIsActive(); i++) {
            outtake.enableSpindexKickerCycle(true, SHOOT_RPM);
            sleep(900);
            spindex.addIndex();
            sleep(150);
        }

        outtake.setRPM(0);
        sleep(150);
    }
}
