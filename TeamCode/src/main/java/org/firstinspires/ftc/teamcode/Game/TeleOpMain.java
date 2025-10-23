package org.firstinspires.ftc.teamcode.Game;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tele-Op Main", group="Linear OpMode")
public class TeleOpMain extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private Intake intake = null;
    private Outtake outtake = null;
    private Limelight limelight = null;
    private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        
        // Initialize subsystems
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        limelight = new Limelight(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(16);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        
        // Start limelight after waitForStart
        limelight.start();

        while (opModeIsActive()) {

            // Mecanum drive control
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            
            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            // Update subsystems
            // Intake control - left trigger (DEACTIVATED)
            // if (gamepad1.left_trigger > 0.1) {
            //     intake.activate();
            // } else {
            //     intake.deactivate();
            // }
            intake.deactivate(); // Always deactivated
            intake.update();

            // Outtake control - right trigger
            if (gamepad1.right_trigger > 0.1) {
                outtake.activate();
            } else {
                outtake.deactivate();
            }
            outtake.update();

            // Additional Telemetry
            //telemetry.addLine("==========================================");
            //telemetry.addLine(bigThree.foward());
            //telemetry.addLine("==========================================");
            limelight.update();
            telemetry.addLine("\n=== DRIVE & INTAKE ===");
            telemetry.addData("Intake Active", intake.isActive());
            telemetry.addData("Outtake Active", outtake.isActive());
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addLine("Intake RPM: " + Double.toString(intake.getRPM(28)));
            telemetry.addLine("Outtake RPM: " + Double.toString(outtake.getRPM(28)));
            telemetry.update();
        }


    }
    public static void main(String[] args) {
        System.out.println("Father Ryan");
    }
}