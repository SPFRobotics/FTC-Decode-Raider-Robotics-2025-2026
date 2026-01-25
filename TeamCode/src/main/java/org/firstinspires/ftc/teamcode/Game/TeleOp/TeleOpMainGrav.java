package org.firstinspires.ftc.teamcode.Game.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFinder;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Extension;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerGrav;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Game.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Resources.MecanumChassis;
import org.firstinspires.ftc.teamcode.Resources.Scroll;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake.OuttakeConfig.*;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

@Disabled
public class TeleOpMainGrav extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    //DcMotor linearSlides= null;

    private Intake intake = null;
    private ElapsedTime masterClock = new ElapsedTime();
    private Outtake outtake = null;
    private KickerGrav kickerGrav = null;
    //Multiplys the motor power by a certain amount to lower or raise the speed of the motors
    private double speedFactor =  1;
    private Limelight limelight = null;
    private MecanumChassis chassis = null;
    private ColorFinder colorFinder = null;

    //Buttons
    private Button outtakeFar = new Button();
    private Button outtakeClose = new Button();
    private Button outtakeSort = new Button();
    private Button triangle = new Button();
    private Button a = new Button();
    private Button centeringButton = new Button();
    private Button fieldCentric = new Button();
    private Button toggleExtension = new Button();

    private Button circle = new Button();// X button for encoder-based centering

    private LedLights rightLED = null;
    private LedLights leftLED = null;
    private Button square = new Button();
    //Color Detection
    ElapsedTime ledClock = new ElapsedTime();
    boolean colorFound = false;
    //IMU
    private IMU imu = null;

    //telemetry
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private double setRPM = 0;
    private int[] rgb = new int[3];
    private int[] hsv = new int[3];
    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor = null;
    private PrintWriter pen = new PrintWriter("/sdcard/outtake.txt", "UTF-8");
    private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");
    private Scroll daddyRyan = new Scroll("Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.");
    public TeleOpMainGrav() throws FileNotFoundException, UnsupportedEncodingException {
    }

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize subsystems
        outtake = new Outtake(hardwareMap, false);
        kickerGrav = new KickerGrav(hardwareMap);
        Extension extension = new Extension(hardwareMap, "kickstand");
        Servo leftLed = hardwareMap.get(Servo.class, "leftLed");
        Servo rightLed = null;
        colorFinder = new ColorFinder(hardwareMap);
        
        //limelight = new Limelight(hardwareMap, telemetry);
        
        // Initialize MecanumChassis for encoder-based centering
        chassis = new MecanumChassis(this);

        //Initialize Telemetry/FTCdashboard

        telemetry.setMsTransmissionInterval(16);
        dashboardTelemetry.setMsTransmissionInterval(16);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        //kicker.up(true);

        // Initialize chassis movement after waitForStart
        // Note: This will reset encoders and initialize IMU
        // The IMU orientation in MecanumChassis may differ from Limelight's orientation
        // If there are issues, you may need to align the IMU orientations
        chassis.initializeMovement();
        
        // Ensure motors are in manual control mode for TeleOp driving
        // This allows direct power control without encoder interference
        chassis.restoreManualControl();

        // Start limelight after waitForStart
        //limelight.start();
        while (opModeIsActive()) {
            hsv = colorFinder.rgbToHSV(rgb[0], rgb[1], rgb[2]);
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

            boolean xButtonPressed = centeringButton.press(gamepad1.cross);
            

            
            // Debug: Show X button state
            telemetry.addData("X Button State", gamepad1.x ? "PRESSED" : "not pressed");
            telemetry.addData("X Button Press Detected", xButtonPressed ? "YES" : "NO");

            // Mecanum drive control - ALWAYS active (except during centering which blocks)
            double y = -gamepad1.left_stick_y*speedFactor;
            double x = gamepad1.left_stick_x*speedFactor;
            double rx = gamepad1.right_stick_x*speedFactor;

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            //double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = 0;
            double frontLeftPower = 0;
            double backLeftPower = 0;
            double frontRightPower = 0;
            double backRightPower = 0;

            /*if (gamepad1.options){
                imu.resetYaw();
            }*/

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
            /*else{
                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;
            }*/

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

            /*if (square.press(gamepad2.square)) {
                    linearSlides.setPower(1.0);
            }
            else if (circle.press(gamepad2.circle)) {
                linearSlides.setPower(0.0);
            }*/

            if (a.press(gamepad2.a)){
                kickerGrav.down();
            }
            else if (triangle.press(gamepad2.y)){
                kickerGrav.up();
            }

            if (setRPM == closeRPM && outtake.getRPM() >= setRPM){
                gamepad2.rumble(100);
            }
            else if (setRPM == farRPM && outtake.getRPM() >= setRPM){
                gamepad2.rumble(100);
            }
            else if (setRPM == sortRPM && outtake.getRPM() >= setRPM){
                gamepad2.rumble(100);
            }
            else{
                gamepad2.stopRumble();
            }

            // Outtake control - right trigger
            if (outtakeFar.press(gamepad2.dpad_up)) {
                setRPM = farRPM;
            }
            if(outtakeClose.press(gamepad2.dpad_down)){
                setRPM = closeRPM;
            }
            if (outtakeSort.press(gamepad2.share)){
                setRPM = sortRPM;
            }
            if (gamepad2.ps) {
                setRPM = 0;
            }
            
            outtake.setRPM(setRPM);

            /*if (colorFinder != null) {
                if (colorFinder.isGreen() && ledClock.milliseconds() >= 500) {
                    leftLED.setGreen();
                    rightLED.setGreen();
                    colorFound = true;
                } else if (colorFinder.isPurple() && ledClock.milliseconds() >= 500) {
                    leftLED.setViolet();
                    rightLED.setViolet();
                    colorFound = true;
                } else {
                    leftLED.turnOFF();
                    rightLED.turnOFF();
                    colorFound = false;
                }
                if (ledClock.milliseconds() >= 500 && !colorFound) {
                    ledClock.reset();
                }
            }*/

            //Kickstand
            if (toggleExtension.toggle(gamepad1.left_bumper && gamepad1.right_bumper)){
                extension.kickStandUp(false);
            }
            else{
                extension.kickStandUp(true);
            }

            // Driver hub
            telemetry.addLine("==========================================");
            telemetry.addLine(bigThree.foward());
            telemetry.addLine("==========================================");
            telemetry.addLine("=== DRIVE & OUTTAKE ===");
            telemetry.addData("Outtake Active", outtake.isActive());
            if (a.getState() == true){
                telemetry.addLine("Kicker Active");
            }
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("Outtake RPM ", outtake.getRPM());
            telemetry.addData("PIDF", outtake.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addLine(Double.toString(outtake.getCurrentCycleTime()));
            telemetry.addData("Rumbleing:", gamepad2.isRumbling());
            telemetry.addLine("=== AUTO-CENTERING ===");

            telemetry.addLine("==========================================");
            telemetry.addLine(daddyRyan.foward());
            telemetry.addLine("==========================================");
            telemetry.update();

            //Dashboard
            dashboardTelemetry.addLine("==========================================");
            dashboardTelemetry.addLine(bigThree.foward());
            dashboardTelemetry.addLine("==========================================");
            dashboardTelemetry.addLine("=== DRIVE & OUTTAKE ===");
            dashboardTelemetry.addData("Outtake Active", outtake.isActive());
            if (a.getState() == true){
                dashboardTelemetry.addLine("Kicker Active");
            }
            dashboardTelemetry.addData("Runtime", runtime.toString());
            dashboardTelemetry.addData("Outtake RPM ", outtake.getRPM());
            dashboardTelemetry.addData("PIDF", outtake.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            dashboardTelemetry.addLine(Double.toString(outtake.getCurrentCycleTime()));
            dashboardTelemetry.addData("Rumbleing:", gamepad2.isRumbling());
            dashboardTelemetry.addLine("HUE: " + hsv[0]);
            dashboardTelemetry.addLine("=== AUTO-CENTERING ===");

            dashboardTelemetry.addLine("==========================================");
            dashboardTelemetry.addLine(daddyRyan.foward());
            dashboardTelemetry.addLine("==========================================");
            dashboardTelemetry.update();

            pen.write((int)runtime.milliseconds() + ":" + (int)outtake.getRPM() + "\n");
        }
        pen.close();
    }
}