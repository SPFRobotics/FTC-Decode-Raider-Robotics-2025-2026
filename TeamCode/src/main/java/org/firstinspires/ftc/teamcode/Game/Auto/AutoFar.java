package org.firstinspires.ftc.teamcode.Game.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Game.TeleOp.Kicker;
import org.firstinspires.ftc.teamcode.Game.TeleOp.Outtake;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

@Autonomous(name="Auto Long")
public class AutoFar extends LinearOpMode {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private Outtake outtake = null;
    private ElapsedTime masterClock = new ElapsedTime();
    private Kicker kicker = null;
    private boolean isActive = false;
    //change

    public void runOpMode() {
        MecanumChassis robot = new MecanumChassis(this);
        robot.initializeMovement();
        outtake = new Outtake(hardwareMap);
        kicker = new Kicker(hardwareMap);
        // Reverse the left motors if needed

        waitForStart();
        robot.rotate(25.0,.1);
        outtake.setRPM(Outtake.OuttakeSpeed.farRPM);
        while (opModeIsActive()){
            if (outtake.getKickerCycleCount() < 3 && masterClock.seconds() >= 5){
                outtake.enableKickerCycle(true);
            }
            else{
                outtake.enableKickerCycle(false);
            }

            if (outtake.getKickerCycleCount() == 3){
                break;
            }
        }

        }
    }

