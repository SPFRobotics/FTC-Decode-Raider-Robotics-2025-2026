package org.firstinspires.ftc.teamcode.Game.Auto.GravFed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.KickerGrav;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

@Disabled
        //(name="Auto Blue Long")
public class AutoFarBlue extends LinearOpMode {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotorEx outtakeMotor = null;
    private ElapsedTime masterClock = new ElapsedTime();
    private KickerGrav kickerGrav = null;
    private Outtake outtake = null;
    private boolean isActive = false;
    FtcDashboard dashboard = null;
    //change

    public void runOpMode() {
        //dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();
        MecanumChassis robot = new MecanumChassis(this);
        robot.initializeMovement();
        outtake = new Outtake(hardwareMap);
        kickerGrav = new KickerGrav(hardwareMap);

        // Reverse the left motors if needed

        waitForStart();
        robot.rotate(20.0,.1);
        outtake.setRPM(farRPM);
        //sleep(5000);


        while (opModeIsActive()) {
            outtake.enableKickerCycle(true, farRPM);

            if (outtake.getKickerCycleCount() == 3) {
                break;
            }
            //System.out.printf(";%.3f;%d;%s%n", getRuntime(), (int)outtake.getRPM(), kicker.getState());
        }
        if (opModeIsActive()){
            robot.rotate(-20.0 ,.1);
            robot.move(.9,"forward",20);
            robot.rotate(90.0,.3);
            robot.move(.9,"forward",30);
            robot.move(.9,"backward",30);
            robot.rotate(-90.0,.3);
            robot.move(.9,"forward",67);
            robot.rotate(25.0,.3);
            //shoot
            robot.move(.9,"backward",47);
            robot.rotate(90.0,.3);
            robot.move(.9,"forward",30);
            robot.move(.9,"backward",30);
            robot.rotate(-90.0,.3);
            robot.move(.9,"forward",47);
            robot.rotate(25.0,.3);
            //shoot
            robot.move(.9,"backward",27);
            robot.rotate(90.0,.3);
            robot.move(.9,"forward",30);
            //intake
            robot.move(.9,"backward",30);
            robot.rotate(-90.0,.3);
            robot.move(.9,"forward",27);
            robot.rotate(25.0,.3);
            //shoot
        }

    }
}

