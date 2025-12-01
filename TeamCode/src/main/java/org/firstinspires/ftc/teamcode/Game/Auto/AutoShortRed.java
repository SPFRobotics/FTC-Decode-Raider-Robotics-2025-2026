package org.firstinspires.ftc.teamcode.Game.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Kicker;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;

@Autonomous(name="Auto Short Red")
public class AutoShortRed extends LinearOpMode {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotorEx outtakeMotor = null;
    private Limelight limelight = null;
    public int motif = -1;

    private ElapsedTime masterClock = new ElapsedTime();
    private Kicker kicker = null;
    private Outtake outtake = null;
    private boolean isActive = false;
    //change

    public void runOpMode() {
        MecanumChassis robot = new MecanumChassis(this);
        robot.initializeMovement();
        outtake = new Outtake(hardwareMap);
        kicker = new Kicker(hardwareMap);
        limelight = new Limelight(hardwareMap);

        // Reverse the left motors if needed

        waitForStart();

        // get obilisk code
        while (motif == -1) motif = limelight.getMotifId();
        robot.move(-.7,"backward",48);
        outtake.setRPM(Outtake.OuttakeSpeed.closeRPM-100);
        sleep(5000);
        while (opModeIsActive()) {
            outtake.enableKickerCycle(true, Outtake.OuttakeSpeed.closeRPM-100);

            if (outtake.getKickerCycleCount()==3 && robot.getWiggleCount() < 2){
                kicker.down(true);
                robot.wiggle();
            }
            else if (robot.getWiggleCount() == 3){
                kicker.up(true);
            }

            if (outtake.getKickerCycleCount() == 4) {
                break;
            }
        }
        if (opModeIsActive()){
            robot.move(.9,"right",20);
        }
    }
}
