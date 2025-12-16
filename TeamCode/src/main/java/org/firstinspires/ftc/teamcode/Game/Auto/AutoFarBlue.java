package org.firstinspires.ftc.teamcode.Game.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Game.TeleOp.Kicker;
import org.firstinspires.ftc.teamcode.Game.TeleOp.Outtake;
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
    private boolean isActive = false;
    FtcDashboard dashboard = null;
    //change

    public void runOpMode() {
            //dashboard = FtcDashboard.getInstance();
            //telemetry = dashboard.getTelemetry();
            MecanumChassis robot = new MecanumChassis(this);
            robot.initializeMovement();
            outtake = new Outtake(hardwareMap);
            kicker = new Kicker(hardwareMap);

            // Reverse the left motors if needed

            waitForStart();
            robot.rotate(20.0,.1);
            outtake.setRPM(Outtake.OuttakeSpeed.farRPM);
            sleep(5000);


            while (opModeIsActive()) {
                outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.farRPM);
                if (outtake.getKickerCycleCount()==3 && robot.getWiggleCount() < 2){
                    kicker.down();
                    robot.wiggle();
                }
                else if (robot.getWiggleCount() == 3){
                    kicker.up();
                }
                if (outtake.getKickerCycleCount() == 4) {
                    break;
                }
                //System.out.printf(";%.3f;%d;%s%n", getRuntime(), (int)outtake.getRPM(), kicker.getState());
            }
            if (opModeIsActive()){
                robot.move(.9,"forward",20);
            }

        }
    }

