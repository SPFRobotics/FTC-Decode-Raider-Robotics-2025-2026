package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFinder;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.LedLights;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;
import org.firstinspires.ftc.teamcode.Resources.Scroll;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake.OuttakeConfig.*;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

@TeleOp(name="Test")
public class Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private Intake intake = null;
    private ElapsedTime masterClock = new ElapsedTime();
    private ElapsedTime ledClock = new ElapsedTime();
    private Outtake outtake = null;
    private KickerSpindex kicker = null;
    //Multiplys the motor power by a certain amount to lower or raise the speed of the motors
    private int ballCount = 0;
    private double speedFactor =  1;
    private boolean colorFound = false;
    //Used to check if all 3 balls have been launched
    private boolean launched = true;
    private Limelight limelight = null;
    private MecanumChassis chassis = null;
    private ColorFinder colorSensor = null;
    //private Extension extension = null;
    private Spindex spindex = null;
    private boolean spindexOuttakeMode = false;

    //Buttons
    private Button outtakeFar = new Button();
    private Button outtakeClose = new Button();
    private Button triangle = new Button();
    private Button a = new Button();
    private Button centeringButton = new Button();

    private Button spindexModeToggle = new Button();
    private Button spindexRightBumper = new Button();
    private Button spindexLeftBumper = new Button();
    //private Button kickstandToggle = new Button();
    private Servo ledRight = null;
    private Servo ledLeft = null;
    private LedLights leftLED = null;
    private LedLights rightLED = null;

    private Button square = new Button();
    //telemetry

    private double setRPM = 0;
    private PrintWriter pen = new PrintWriter("/sdcard/outtake.txt", "UTF-8");
    private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");
    private Scroll daddyRyan = new Scroll("Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.");

    public Test() throws FileNotFoundException, UnsupportedEncodingException {
    }

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        ledLeft = hardwareMap.get(Servo.class, "leftLED");
        ledRight = hardwareMap.get(Servo.class, "rightLED");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize subsystems
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap, false);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        colorSensor = new ColorFinder(hardwareMap);
        Spindex spindex = new Spindex(hardwareMap);


        // Initialize LED Lights
        leftLED = new LedLights("leftLED", hardwareMap);
        rightLED = new LedLights("rightLED", hardwareMap);

        //Initialize Telemetry

        telemetry.setMsTransmissionInterval(16);
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            int[] rgb = colorSensor.getColor();
            int[] hsv = colorSensor.rgbToHSV(rgb[0], rgb[1], rgb[2]);

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

            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeftDrive.setPower(y + x + rx);
            backLeftDrive.setPower(y - x + rx);
            frontRightDrive.setPower(y + x - rx);
            backRightDrive.setPower(y - x - rx);

            // Intake toggle on Square button
            boolean intakeActive = square.toggle(gamepad1.right_trigger > 0);
            if (intakeActive) {
                if (gamepad1.left_trigger > 0){
                    intake.setPower(-1);
                }
                else {
                    intake.setPower(1);
                }
            }
            else {
                intake.setPower(0);
            }

            // Spindex mode toggle and position cycling
            if (spindexRightBumper.press(gamepad1.right_bumper)) {
                spindex.addIndex();
            }
            if (spindexLeftBumper.press(gamepad1.left_bumper)) {
                spindex.subtractIndex();
            }
            spindexOuttakeMode = spindexModeToggle.toggle(gamepad1.circle);

            if (kicker.automate(gamepad1.crossWasPressed() && spindexOuttakeMode)){
                //spindex.addIndex();
            }

            if (spindexOuttakeMode){
                spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], true);
            }
            else{
                spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], true);
            }

            //Controls spindex loading using the color sensor
            if (colorSensor.getDistance() <= 5.7 && spindex.getPower() == 0 && ballCount < 3) {
                spindex.addIndex();
                ballCount++;
            }


            if (setRPM == closeRPM && outtake.getRPM() >= setRPM){
                gamepad1.rumble(100);
            }
            else if (setRPM == farRPM & outtake.getRPM() >= setRPM){
                gamepad1.rumble(100);
            }
            else{
                gamepad1.stopRumble();
            }

            // Outtake control - right trigger
            if (outtakeFar.press(gamepad1.dpad_up)) {
                setRPM = farRPM;
            }
            if(outtakeClose.press(gamepad1.dpad_down)){
                setRPM = closeRPM;
            }
            if (gamepad1.ps) {
                setRPM = 0;
            }

            outtake.setRPM(setRPM);


            // Driver Hub
            telemetry.addLine("==========================================");
            telemetry.addLine(bigThree.foward());
            telemetry.addLine("==========================================");
            telemetry.addLine("=== DRIVE & INTAKE ===");
            telemetry.addData("Outtake Active", outtake.isActive());
            if (a.getState() == true){
                telemetry.addLine("Kicker Active");
            }
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("Outtake RPM: ", outtake.getRPM());
            telemetry.addData("PIDF", outtake.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Rumbling:", gamepad2.isRumbling());
            telemetry.addLine("=== SPINDEX ===");
            telemetry.addData("Mode", spindexOuttakeMode ? "OUTTAKE" : "INTAKE");
            telemetry.addData("Index", spindex.getIndex());
            telemetry.addData("Spindex", spindex.getPos());
            telemetry.addData("Voltage", spindex.getVoltage());
            telemetry.addData("Distance", colorSensor.getDistance());
            telemetry.addData("Hue:", hsv[0]);
            telemetry.addData("Current", spindex.getAmps());
            telemetry.addLine("==========================================");
            telemetry.addLine(daddyRyan.foward());
            telemetry.addLine("==========================================");
            telemetry.update();
        }
    }
}