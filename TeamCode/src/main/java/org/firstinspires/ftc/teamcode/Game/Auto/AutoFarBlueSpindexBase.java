package org.firstinspires.ftc.teamcode.Game.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

@Autonomous(name = "Auto Far Blue Spindex (Base)", group = "Autonomous")
public class AutoFarBlueSpindexBase extends LinearOpMode {

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
        shoot3();
        unAim110();

        goToLine1();
        intake3OnLine();
        returnToShootingFromLine1();
        aim110();
        shoot3();
        unAim110();

        goToLine2();
        intake3OnLine();
        returnToShootingFromLine2();
        aim110();
        shoot3();
        unAim110();

        goToLine3();
        intake3OnLine();
        returnToShootingFromLine3();
        aim110();
        shoot3();
        unAim110();

        leave();
    }

    private void aim110() {
        robot.rotate(20.0, 0.25);
    }

    private void unAim110() {
        robot.rotate(-20.0, 0.25);
    }

    private void shoot3() {
        outtake.resetKickerCycle();
        outtake.setRPM(SHOOT_RPM);

        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive() && t.seconds() < 6.0 && outtake.getKickerCycleCount() < 3) {
            outtake.enableSpindexKickerCycle(true, SHOOT_RPM);
            idle();
        }

        outtake.setRPM(0);
        outtake.resetKickerCycle();
        sleep(150);
    }

    private void intake3OnLine() {
        intake.intakeOn();

        robot.move(0.35, "forward", 28);
        spindex.addIndex();
        sleep(150);

        robot.move(0.35, "forward", 28);
        spindex.addIndex();
        sleep(150);

        robot.move(0.35, "forward", 28);
        spindex.addIndex();
        sleep(150);

        intake.intakeOff();

        robot.move(0.6, "backward", 84);
        sleep(100);
    }

    private void goToLine1() {
        robot.move(0.8, "forward", 20);
        robot.move(0.8, "left", 18);
        robot.move(0.75, "forward", 18);
    }

    private void returnToShootingFromLine1() {
        robot.move(0.8, "backward", 18);
        robot.move(0.8, "right", 18);
        robot.move(0.8, "backward", 20);
    }

    private void goToLine2() {
        robot.move(0.9, "left", 40);
        robot.move(0.75, "forward", 18);
    }

    private void returnToShootingFromLine2() {
        robot.move(0.8, "backward", 18);
        robot.move(0.9, "right", 40);
    }

    private void goToLine3() {
        robot.move(0.9, "left", 62);
        robot.move(0.75, "forward", 18);
    }

    private void returnToShootingFromLine3() {
        robot.move(0.8, "backward", 18);
        robot.move(0.9, "right", 62);
    }

    private void leave() {
        robot.move(0.9, "left", 34);
        robot.move(0.9, "forward", 8);
        outtake.setRPM(0);
        intake.intakeOff();
    }
}