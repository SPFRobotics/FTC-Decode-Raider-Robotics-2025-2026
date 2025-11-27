package org.firstinspires.ftc.teamcode.Game.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Kicker;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

@Autonomous(name="Auto Blue Long")
public class AutoFarBlue extends LinearOpMode {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;         
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotorEx outtakeMotor = null;
    private ElapsedTime masterClock = new ElapsedTime();
    private ElapsedTime logTime = new ElapsedTime();
    private Telemetry telemetry = null;
    FtcDashboard dashboard = null;
    private Kicker kicker = null;
    private Outtake outtake = null;
    private boolean isActive = false;
    private boolean logTimeReset = false;
    private String outtakeRPMGraph;
    private PrintWriter pen = new PrintWriter("/sdcard/outtake.txt", "UTF-8");
    private ElapsedTime runtime = new ElapsedTime();
    private boolean speedReached = true;

    public AutoFarBlue() throws FileNotFoundException, UnsupportedEncodingException {
    }
    //change

    public void runOpMode() {
            dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
            telemetry.addData("Outake RPM: ", 0);
            telemetry.setMsTransmissionInterval(1);
            telemetry.update();
            MecanumChassis robot = new MecanumChassis(this);
            robot.initializeMovement();
            outtake = new Outtake(hardwareMap);
            kicker = new Kicker(hardwareMap);

            // Reverse the left motors if needed

            waitForStart();
            robot.rotate(20.0,.1);
            outtake.setRPM(Outtake.OuttakeSpeed.farRPM);
            masterClock.reset();
            while (opModeIsActive()) {
                if (outtake.getRPM() == 3200) {
                    speedReached = true;
                }
                if (speedReached){
                    outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.farRPM);
                    if (outtake.getKickerCycleCount() == 4) {
                        break;
                    }
                }
                //System.out.printf(";%.3f;%d;%s%n", getRuntime(), (int)outtake.getRPM(), kicker.getState());
                telemetry.addData("Outake RPM: ", outtake.getRPM());
                telemetry.addData("PIDF: ", outtake.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
                telemetry.update();
                if (!logTimeReset){
                    logTime.reset();
                    logTimeReset = true;
                }
                pen.write((int)logTime.milliseconds() + ":" + (int)outtake.getRPM() + ":" + Kicker.getState() + "\n");

            }
            pen.close();
            if (opModeIsActive()){
                //robot.move(.9,"forward",20);
            }
            telemetry.update();
    }}

