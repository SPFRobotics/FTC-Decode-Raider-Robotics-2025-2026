package org.firstinspires.ftc.teamcode.Game.TeleOp;

import android.os.Environment;
import android.util.PrintWriterPrinter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Game.ColorModels;
import org.firstinspires.ftc.teamcode.Game.Intake;
import org.firstinspires.ftc.teamcode.Game.Kicker;
import org.firstinspires.ftc.teamcode.Game.Limelight;
import org.firstinspires.ftc.teamcode.Game.Outtake;
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
    private Limelight limelight = null;
    private MecanumChassis chassis = null;

    //Buttons
    private Button outtakeFar = new Button();
    private Button outtakeClose = new Button();
    private Button triangle = new Button();
    private Button a = new Button();
    private Button centeringButton = new Button(); // X button for encoder-based centering

    private Servo ledRight = null;
    private Servo ledLeft = null;

    //telemetry
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();;
    private double setRPM = 0;
    private ColorModels conversion = new ColorModels();
    private ColorSensor colorSensor = null;
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
        ledLeft = hardwareMap.get(Servo.class, "leftLED");
        ledRight = hardwareMap.get(Servo.class, "rightLED");

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
        limelight = new Limelight(hardwareMap, telemetry);
        
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
        // The IMU orientation in MecanumChassis may differ from Limelight's orientation
        // If there are issues, you may need to align the IMU orientations
        chassis.initializeMovement();
        
        // Ensure motors are in manual control mode for TeleOp driving
        // This allows direct power control without encoder interference
        chassis.restoreManualControl();

        // Start limelight after waitForStart
        limelight.start();
        ledLeft.setPosition(0.5);
        ledRight.setPosition(0.5);

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

            // Update limelight (needed for centering calculations)
            limelight.update();

            // Encoder-based auto-centering (X button on gamepad1)
            // CURRENT: Single click starts full centering sequence (runs to completion)
            // To change to hold-to-center, replace press() with direct gamepad1.x check
            
            // Option 1: Single click (current) - click once, centers fully, then stops
            boolean xButtonPressed = centeringButton.press(gamepad1.cross);
            
            // Option 2: Hold-to-center (uncomment to use) - hold X, centers while held, stops when released
            // boolean xButtonPressed = gamepad1.x;
            
            /*if (xButtonPressed) {
                // X button was just pressed - trigger encoder-based centering
                telemetry.addData("!!! CENTERING DEBUG !!!", "X button pressed!");
                telemetry.addData("Has Valid Target", limelight.hasValidTarget());
                telemetry.update();
                
                // Get distance to target from Limelight
                double distanceToTarget = limelight.getDistanceToTarget();
                telemetry.addData("Distance to Target", String.format("%.1f", distanceToTarget));
                telemetry.update();
                
                if (distanceToTarget > 0 && limelight.hasValidTarget()) {
                    telemetry.addData("Limelight Center", "Starting full centering...");
                    telemetry.update();
                    // Full centering: rotate to face target, then strafe to center position
                    boolean success = limelight.centerOnTargetFull(chassis, 0.4, 0.4, distanceToTarget);
                    telemetry.addData("Centering Result", success ? "Success" : "Failed");
                    telemetry.addData("Limelight Center", "Centered! Ready to shoot.");
                } else if (limelight.hasValidTarget()) {
                    telemetry.addData("Limelight Center", "Starting rotation only...");
                    telemetry.update();
                    // If we can't get distance, just rotate to face the target
                    boolean success = limelight.centerOnTarget(chassis, 0.4, 0);
                    telemetry.addData("Centering Result", success ? "Success" : "Failed");
                    telemetry.addData("Limelight Center", "Rotated to face target");
                } else {
                    telemetry.addData("Limelight Center", "No target detected - cannot center");
                    telemetry.addData("Limelight Valid", limelight.hasTarget());
                }
                
                // CRITICAL: Restore motors to manual control mode after centering
                // This ensures normal driving works immediately after centering
                chassis.restoreManualControl();
                
                // Double-check: explicitly set all motors to manual mode
                frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                
                telemetry.addData("Motors Restored", "Manual control mode");
                telemetry.update();
                sleep(100); // Brief pause to see telemetry
            }*/
            
            // Debug: Show X button state
            telemetry.addData("X Button State", gamepad1.x ? "PRESSED" : "not pressed");
            telemetry.addData("X Button Press Detected", xButtonPressed ? "YES" : "NO");

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
            telemetry.addLine("=== AUTO-CENTERING ===");
            telemetry.addData("Press X to Center", "Full (Rotate + Strafe)");
            telemetry.addData("Has Target", limelight.hasValidTarget());
            telemetry.addData("Centered", limelight.isCentered());
            limelight.getAprilTagCount();
            if (limelight.hasValidTarget()) {
                telemetry.addData("Tx (deg)", String.format("%.2f", limelight.getTx()));
                telemetry.addData("Rotation Angle", String.format("%.2f", limelight.getCenteringRotationAngle()));
                double distance = limelight.getDistanceToTarget();
                if (distance > 0) {
                    telemetry.addData("Distance (in)", String.format("%.1f", distance));
                } else {
                    telemetry.addData("Distance", "Unknown");
                }
            }
            //telemetry.addData("Color ", );
            telemetry.addLine("==========================================");
            telemetry.addLine(daddyRyan.foward());
            telemetry.addLine("==========================================");
            telemetry.update();
            pen.write((int)runtime.milliseconds() + ":" + (int)outtake.getRPM() + "\n");
        }
        pen.close();
    }
}