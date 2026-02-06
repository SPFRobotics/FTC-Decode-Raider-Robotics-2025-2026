package org.firstinspires.ftc.teamcode.Game.Auto.GravFed;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.KickerGrav;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;


@Disabled
        //(name="Auto Short Red")
public class AutoShortRed extends LinearOpMode {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotorEx outtakeMotor = null;
    private ElapsedTime masterClock = new ElapsedTime();
    private KickerGrav kickerGrav = null;
    private Outtake outtake = null;
    private Limelight limelight = null;
    public int motif = -1;
    private PrintWriter pen = new PrintWriter("/sdcard/outtake.txt", "UTF-8");

    public AutoShortRed() throws FileNotFoundException, UnsupportedEncodingException {
    }

    private boolean isActive = false;

    public void runOpMode() {
        MecanumChassis robot = new MecanumChassis(this);
        robot.initializeMovement();
        outtake = new Outtake(hardwareMap);
        kickerGrav = new KickerGrav(hardwareMap);
        limelight = new Limelight(hardwareMap);

        telemetry.setMsTransmissionInterval(16);


        //limelight.start();

        waitForStart();

        robot.move(-.7, "backward", 48);
        outtake.setRPM(closeRPM);
        //sleep(3000);

        while (opModeIsActive()) {
            outtake.enableKickerCycle(true, closeRPM);

            if (outtake.getKickerCycleCount() == 3) {
                break;
            }
            telemetry.addData("Interval", outtake.getInverval());
            telemetry.addData("RPM", outtake.getRPM());
            telemetry.addData("Launched", outtake.launched);
            telemetry.update();
            pen.write((int)masterClock.milliseconds() + ":" + (int)outtake.getRPM() + ":" + KickerGrav.getState() + "\n");
        }
        masterClock.reset();

        if (opModeIsActive()) {
            robot.move(.9, "right", 20);
        }
        pen.close();
    }
}
