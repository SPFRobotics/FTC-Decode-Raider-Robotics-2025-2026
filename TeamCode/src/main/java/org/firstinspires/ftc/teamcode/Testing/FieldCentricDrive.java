package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.closeRPM;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.farRPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.KickstandServo;
import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Subsystems.UpdateSpindex;


@TeleOp(name = "Field Centric TeleOp")
public class FieldCentricDrive extends LinearOpMode {
    // Speed multiplier for motors
    private double speedFactor = 1;

    // Buttons
    private Button spindexModeToggle = new Button();
    private Button spindexRightBumper = new Button();
    private Button spindexLeftBumper = new Button();
    private Button kickstandButton = new Button();
    private Button intakeButton = new Button();
    private Button autoLoad = new Button();

    private double setRPM = 0;
    boolean displayDash = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare drive motors
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Reverse the right side motors for field centric drive
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders for odometry pods
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run without encoder for manual control
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        // Initialize subsystems
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap, true);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        ColorFetch colorSensor = new ColorFetch(hardwareMap);
        Spindex spindex = new Spindex(hardwareMap);
        UpdateSpindex updateSpindex = new UpdateSpindex(spindex);
        KickstandServo kickstand = new KickstandServo(hardwareMap);
        LedLights leds = new LedLights(hardwareMap);

        Telemetry driverHub = telemetry;
        FtcDashboard dash = FtcDashboard.getInstance();

        // Set auto load to true as default
        autoLoad.changeState(true);

        // Cycle LED colors during init
        while (opModeInInit()) {
            leds.cycleColors(10);
        }

        waitForStart();

        leds.setColor(leds.RED, false);

        if (opModeIsActive()) {
            updateSpindex.start();
        }

        if (isStopRequested()) return;

        ElapsedTime loopTime = new ElapsedTime();

        while (opModeIsActive()) {
            loopTime.reset();

            /*************************************Field Centric Drive Control**************************************/
            // Allows speed to be halved with triggers
            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) {
                speedFactor = 0.5;
            } else {
                speedFactor = 1;
            }

            double y = -gamepad1.left_stick_y * speedFactor; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * speedFactor;
            double rx = gamepad1.right_stick_x * speedFactor;

            // Reset IMU yaw with options button (start on Xbox controllers)
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);
            /**********************************************************************************************/

            /*****************************Intake System************************************/
            boolean intakeActive = intakeButton.toggle(gamepad1.right_bumper);
            if (intakeActive && !gamepad1.left_bumper) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
            /******************************************************************************/

            /**********Spindex mode toggle and position cycling***********/
            if (spindexRightBumper.press(gamepad2.right_bumper)) {
                if (!spindex.isOuttakeing()) {
                    autoLoad.changeState(false);
                }
                spindex.addIndex();
            }
            if (spindexLeftBumper.press(gamepad2.left_bumper)) {
                if (!spindex.isOuttakeing()) {
                    autoLoad.changeState(false);
                }
                spindex.subtractIndex();
            }
            // Sets either intake or outtake mode
            spindex.setMode(spindexModeToggle.toggle(gamepad2.circle));
            /************************************************************/

            /*********************Kicker and index emptying logic**********************/
            boolean crossWasPressed = gamepad2.crossWasPressed();
            kicker.automate(crossWasPressed && spindex.isOuttakeing());
            if (crossWasPressed && spindex.isOuttakeing() && outtake.getPower() != 0) {
                spindex.clearBall(spindex.getIndex());
            }
            /**************************************************************************/

            spindex.setAutoLoadMode(autoLoad.toggle(gamepad2.triangle) && !spindex.isOuttakeing());
            spindex.autoLoad(colorSensor);

            // Controls gamepad rumble and LED colors based on outtake RPM
            if (setRPM == closeRPM && outtake.getRPM() >= setRPM) {
                gamepad2.rumble(100);
                leds.setColor(leds.GREEN);
            } else if (setRPM == farRPM & outtake.getRPM() >= setRPM) {
                gamepad2.rumble(100);
                leds.setColor(leds.GREEN);
            } else {
                gamepad2.stopRumble();
                leds.setColor(leds.RED);
            }

            // Outtake control - dpad
            if (gamepad2.dpad_up) {
                setRPM = farRPM;
            } else if (gamepad2.dpad_down) {
                setRPM = closeRPM;
            } else if (gamepad2.touchpad) {
                setRPM = 0;
            }
            outtake.setRPM(setRPM);

            // Kickstand control
            if (kickstandButton.toggle(gamepad1.share)) {
                kickstand.updatePos(KickstandServo.KickstandServoConfig.up);
            } else {
                kickstand.setPower(0);
            }

            // Telemetry
            telemetry.addLine("==========================================");
            telemetry.addData("Loop Time", loopTime.milliseconds());
            telemetry.addData("Spindex Updater Loop Time", spindex.getThreadLoopTime());
            telemetry.addLine("------------------------------------------");
            telemetry.addData("Bot Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Speed Factor", speedFactor);
            telemetry.addLine("------------------------------------------");
            telemetry.addData("Spindex Index", spindex.getIndex());
            telemetry.addData("Slot Status", spindex.getSlotStatus()[0] + " " + spindex.getSlotStatus()[1] + " " + spindex.getSlotStatus()[2]);
            telemetry.addData("Color", colorSensor.getHue());
            telemetry.addData("At Target?", spindex.atTarget());
            telemetry.addData("Spindex Power", spindex.getPower());
            telemetry.addData("Automated Loading", spindex.isAutoLoading());
            telemetry.addData("Outtaking?", spindex.isOuttakeing());
            telemetry.addData("Kickstand Pos", kickstand.getPosition());
            telemetry.addData("Kickstand Voltage", kickstand.getVoltage());
            telemetry.addLine("------------------------------------------");
            telemetry.addData("Distance", colorSensor.getDistance());
            telemetry.addData("Right Pod", backRightDrive.getCurrentPosition());
            telemetry.addData("Left Pod", backLeftDrive.getCurrentPosition());
            telemetry.addData("Strafe Pod", frontRightDrive.getCurrentPosition());
            telemetry.addLine("==========================================");
            telemetry.update();
        }

        // Tell spindex thread to end execution
        spindex.exitProgram();
    }
}
