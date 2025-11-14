package org.firstinspires.ftc.teamcode.Game.TeleOp;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Kicker;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;
import org.firstinspires.ftc.teamcode.Resources.Scroll;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;


@TeleOp(name="Tele-Op Main")
public class TeleOpMain extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;



    private Intake intake = null;
    private Outtake outtake = null;
    private Kicker kicker = null;
    //Multiplys the motor power by a certain amount to lower or raise the speed of the motors
    private double speedFactor =  1;
    private MecanumChassis chassis = null;

    //Buttons
    private Button outtakeFar = new Button();
    private Button outtakeClose = new Button();
    private Button triangle = new Button();
    private Button a = new Button();

    //telemetry
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();;
    private double setRPM = 0;
    private PrintWriter pen = new PrintWriter("/sdcard/outtake.txt", "UTF-8");
    private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");
    private Scroll daddyRyan = new Scroll("Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.");

    public TeleOpMain() throws FileNotFoundException, UnsupportedEncodingException {
    }

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize subsystems
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        kicker = new Kicker(hardwareMap);

        //extension = new Extension(hardwareMap);
        
        // Initialize MecanumChassis for encoder-based centering
        chassis = new MecanumChassis(this);

        //Initialize Telemetry/FTCdashboard

        telemetry.setMsTransmissionInterval(16);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        kicker.up();

        // Initialize chassis movement after waitForStart
        // Note: This will reset encoders and initialize IMU
        chassis.initializeMovement();
        
        // Ensure motors are in manual control mode for TeleOp driving
        // This allows direct power control without encoder interference
        chassis.restoreManualControl();

        while (opModeIsActive()) {
            // Always ensure motors are in manual control mode for normal driving
            // This ensures they respond to direct power commands
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
                speedFactor = 0.5;
            }
            else{
                speedFactor = 1;
            }


            // Mecanum drive control - ALWAYS active (except during centering which blocks)
            double y = -gamepad1.left_stick_y*speedFactor;
            double x = gamepad1.left_stick_x*speedFactor;
            double rx = gamepad1.right_stick_x*speedFactor;
            
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            //Automate the kicker
            /*outtake.automate(a.toggle(gamepad2.a));
            if (a.getState() && rumbled == false){
                rumbled = true;
                gamepad2.rumbleBlips(1);
            }
            else if (!a.getState() && rumbled == true){
                rumbled = false;
                gamepad2.rumbleBlips(2);
            }*/

            if (a.press(gamepad2.a)){
                kicker.down();
            }
            else if (triangle.press(gamepad2.y)){
                kicker.up();
            }

            if (setRPM == Outtake.OuttakeSpeed.closeRPM && outtake.getRPM() >= setRPM){
                gamepad2.rumble(100);
            }
            else if (setRPM == Outtake.OuttakeSpeed.farRPM & outtake.getRPM() >= setRPM){
                gamepad2.rumble(100);
            }
            else{
                gamepad2.stopRumble();
            }

            // Outtake control - right trigger
            if (outtakeFar.press(gamepad2.dpad_up)) {
                setRPM = Outtake.OuttakeSpeed.farRPM;
            }
            if(outtakeClose.press(gamepad2.dpad_down)){
                setRPM = Outtake.OuttakeSpeed.closeRPM;
            }
            if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0){
                setRPM = Outtake.OuttakeSpeed.reverseRPM;
            }
            if (gamepad2.ps) {
                setRPM = 0;
            }
            
            outtake.setRPM(setRPM);

            // Additional Telemetry
            telemetry.addLine("==========================================");
            telemetry.addLine(bigThree.foward());
            telemetry.addLine("==========================================");
            telemetry.addLine("=== DRIVE & INTAKE ===");
            telemetry.addData("Intake Active", intake.isActive());
            telemetry.addData("Outtake Active", outtake.isActive());
            if (a.getState() == true){
                telemetry.addLine("Kicker Active");
            }
            telemetry.addData("Runtime", runtime.toString());
            //telemetry.addLine("Intake RPM: " + Double.toString(intake.getRPM(28)));
            telemetry.addData("Outtake RPM: ", outtake.getRPM());
            telemetry.addData("PIDF", outtake.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addLine(Double.toString(outtake.getCurrentCycleTime()));
            telemetry.addData("Rumbleing:", gamepad2.isRumbling());
            telemetry.addData("File Dir: ", Environment.getDataDirectory());
            telemetry.addLine("==========================================");
            telemetry.addLine(daddyRyan.foward());
            telemetry.addLine("==========================================");
            telemetry.update();
            pen.write((int)runtime.milliseconds() + ":" + (int)outtake.getRPM() + "\n");
        }
        pen.close();
    }
}