package org.firstinspires.ftc.teamcode.Game.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFinder;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Kicker;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

@Autonomous(name="Auto Blue Long")
public class AutoFarBlue extends LinearOpMode {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotorEx outtakeMotor = null;
    private ElapsedTime masterClock = new ElapsedTime();
    private Kicker kicker = null;
    private Outtake outtake = null;
    private Limelight limelight = null;
    public int motif = -1;
    private ColorFinder colorFinder = null;
    private int kickerCycleCount = 0;

    private boolean isActive = false;

    public void runOpMode() {
        MecanumChassis robot = new MecanumChassis(this);
        robot.initializeMovement();
        outtake = new Outtake(hardwareMap);
        kicker = new Kicker(hardwareMap);
        limelight = new Limelight(hardwareMap);

        limelight.start();


        waitForStart();


        while (motif == -1) motif = limelight.getMotifId();
        limelight.getMotifId();
        robot.rotate(20.0, .1);
        outtake.setRPM(Outtake.OuttakeSpeed.farRPM);
        sleep(3000);

        while (opModeIsActive()) {
            int result = limelight.getMotifId();

            if (result == (21)) {

                if (kickerCycleCount == 1 || kickerCycleCount == 2 && colorFinder.isPurple()) {
                    outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.farRPM);
                    kicker.down(true);
                } else if (kickerCycleCount == 0 && colorFinder.isGreen()) {

                    outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.farRPM);
                    kicker.down(true);

                } else {
                    outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.sortRPM);
                    kicker.down(true);
                }
            }


            if (result == (22)) {

                if (kickerCycleCount == 0 || kickerCycleCount == 2 && colorFinder.isPurple()) {
                    outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.farRPM);
                    kicker.down(true);
                } else if (kickerCycleCount == 1 && colorFinder.isGreen()) {

                    outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.farRPM);
                    kicker.down(true);

                } else {
                    outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.sortRPM);
                    kicker.down(true);
                }

                if (result == (23)) {

                    if (kickerCycleCount == 0 || kickerCycleCount == 1 && colorFinder.isPurple()) {
                        outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.farRPM);
                        kicker.down(true);
                    } else if (kickerCycleCount == 2 && colorFinder.isGreen()) {

                        outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.farRPM);
                        kicker.down(true);

                    } else {
                        outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.sortRPM);
                        kicker.down(true);
                    }
                }
            }

            if (opModeIsActive() && kickerCycleCount==3) {
                robot.move(.9, "forward", 20);
            }
            telemetry.addData("April Tag ID", motif);
            telemetry.update();
        }
    }
}