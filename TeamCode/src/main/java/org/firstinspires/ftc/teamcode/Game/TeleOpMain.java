package org.firstinspires.ftc.teamcode.Game;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.Scroll;


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
    private boolean rumbled = false;
    private Extension extension = null;
    private boolean zeroKicker = false;
    //Multiplys the motor power by a certain amount to lower or raise the speed of the motors
    private double speedFactor =  1;
    //private Limelight limelight = null;

    private Button outtakeFar = new Button();
    private Button outtakeClose = new Button();
    private Button a = new Button();
    //private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");
    //private Scroll daddyRyan = new Scroll("Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.");
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
        //limelight = new Limelight(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(16);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        kicker.up();
        
        // Start limelight after waitForStart
        //limelight.start();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
                speedFactor = 0.5;
            }
            else{
                speedFactor = 1;
            }

            // Mecanum drive control
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

            if (a.toggle(gamepad2.a)){
                kicker.down();
            }
            else{
                kicker.up();
            }

            // Outtake control - right trigger
            if (outtakeFar.press(gamepad2.dpad_up)) {
                outtake.setRPM(Outtake.OuttakeSpeed.farRPM);
            }
            if(outtakeClose.press(gamepad2.dpad_down)){
                outtake.setRPM(Outtake.OuttakeSpeed.closeRPM);
            }
            if (gamepad2.touchpad) {
                outtake.setRPM(0);
            }

            // Additional Telemetry
            telemetry.addLine("==========================================");
            //telemetry.addLine(bigThree.foward());
            telemetry.addLine("==========================================");
            //limelight.update();
            telemetry.addLine("=== DRIVE & INTAKE ===");
            telemetry.addData("Intake Active", intake.isActive());
            telemetry.addData("Outtake Active", outtake.isActive());
            if (a.getState() == true){
                telemetry.addLine("Kicker Active");
            }
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addLine("Intake RPM: " + Double.toString(intake.getRPM(28)));
            telemetry.addLine("Outtake RPM: " + Double.toString(outtake.getRPM(28)));
            telemetry.addLine(Double.toString(outtake.getCurrentCycleTime()));
            telemetry.addLine("==========================================");
            //telemetry.addLine(daddyRyan.foward());
            telemetry.addLine("==========================================");
            telemetry.update();
        }


    }
    public static void main(String[] args) {
        System.out.println("Father Ryan");
    }
}