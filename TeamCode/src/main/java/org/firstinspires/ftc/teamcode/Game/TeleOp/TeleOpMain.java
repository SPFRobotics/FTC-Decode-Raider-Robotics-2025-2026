package org.firstinspires.ftc.teamcode.Game.TeleOp;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.closeRPM;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.farRPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Resources.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.HuskyLensController;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.KickstandServo;
import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.UpdateSpindex;
import org.firstinspires.ftc.teamcode.Resources.Button;

import java.util.Arrays;

@TeleOp(name="Tele-Op Main")
public class TeleOpMain extends LinearOpMode {
    private Follower follower;
    private Intake intake = null;
    private Outtake outtake = null;
    private KickerSpindex kicker = null;
    private Turret turret = null;

    //Multiplys the motor power by a certain amount to lower or raise the speed of the motors
    private double speedFactor =  1;

    //Buttons
    private Button spindexModeToggle = new Button();
    private Button spindexRightBumper = new Button();
    private Button spindexLeftBumper = new Button();
    private Button kickstandButton = new Button();

    private Button intakeButton = new Button();
    private Button autoLoad = new Button();
    private Button fieldholdButton = new Button();
    private double setRPM = 0;
    ElapsedTime intakeReverseTimer = new ElapsedTime();

    private Pose currentPose;

    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashBoardTele = dash.getTelemetry();
    MultipleTelemetry multiTelemetry = new MultipleTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Always ensure motors are in manual control mode for normal driving
        // This ensures they respond to direct power commands
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize subsystems
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap, true);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        ColorFetch colorSensor = new ColorFetch(hardwareMap);
        Spindex spindex = new Spindex(hardwareMap);
        UpdateSpindex updateSpindex = new UpdateSpindex(spindex);
        KickstandServo kickstand = new KickstandServo(hardwareMap);
        LedLights leds = new LedLights(hardwareMap);
        //HuskyLensController huskyLens = new HuskyLensController(hardwareMap);
        turret = new Turret(hardwareMap, true);

        //Pedro Pathing for turret
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.update();
        follower.startTeleopDrive();


        //Set autoload and launch to true as default
        autoLoad.changeState(true);
        while (opModeInInit()){
            leds.cycleColors(10);
        }
        waitForStart();

        ElapsedTime loopTime = new ElapsedTime();
        multiTelemetry.addTelemetry(telemetry);
        multiTelemetry.addTelemetry(dashBoardTele);
        multiTelemetry.setMsTransmissionInterval(16);
        while (opModeIsActive()) {
            loopTime.reset();

            //Update robot location (Pedro Pathing)
            follower.update();
            currentPose = follower.getPose();
            //Holds robot position when enabled

            /*************************************Drive Train Control**************************************/
            //Allows speed to be halved
            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
                speedFactor = 0.5;
            }
            else{
                speedFactor = 1;
            }

            //Using Pedro Pathing for Tele-Op drive
            follower.setTeleOpDrive(-gamepad1.left_stick_y * speedFactor, gamepad1.left_stick_x * speedFactor, gamepad1.right_stick_x * speedFactor, true); // Remember, Y stick is reversed!


            /**********************************************************************************************/

            /*****************************Intake System************************************/
            boolean intakeActive = intakeButton.toggle(gamepad1.right_bumper);
            if (intakeActive && !gamepad1.left_bumper) {
                intake.intakeOn(true);
            }
            else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);

            }
            /******************************************************************************/

            /**********Spindex mode toggle and position cycling***********/
            if (spindexRightBumper.press(gamepad2.right_bumper)) {
                if (!spindex.isOuttakeing()){
                    autoLoad.changeState(false);
                }
                spindex.addIndex();
            }
            if (spindexLeftBumper.press(gamepad2.left_bumper)) {
                if (!spindex.isOuttakeing()){
                    autoLoad.changeState(false);
                }
                spindex.subtractIndex();
            }
            //Sets either intake or outtake mode
            spindex.setMode(spindexModeToggle.toggle(gamepad2.circle));
            /************************************************************/

            /*********************Kicker and index emptying logic**********************/
            boolean crossWasPressed = gamepad2.crossWasPressed();
            kicker.automate(crossWasPressed && spindex.isOuttakeing());
            if (crossWasPressed && spindex.isOuttakeing() && outtake.getPower() != 0){
                spindex.clearBall(spindex.getIndex());
            }
            /**************************************************************************/

            spindex.setAutoLoadMode(autoLoad.toggle(gamepad2.triangle) && !spindex.isOuttakeing());
            spindex.autoLoad(colorSensor);

            if (spindex.isOuttakeing()){
                spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], 4);
                leds.setColor(leds.GREEN, false);
            }
            else{
                spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], 4);
                leds.setColor(leds.BLUE, false);
            }

            //Controls gamepad rumble
            if (setRPM == closeRPM && outtake.getRPM() >= setRPM){
                gamepad2.rumble(100);
                //leds.setColor(leds.GREEN);
            }
            else if (setRPM == farRPM & outtake.getRPM() >= setRPM){
                gamepad2.rumble(100);
                //leds.setColor(leds.GREEN);
            }
            else{
                gamepad2.stopRumble();
                //leds.setColor(leds.RED);
            }

            // Outtake control
            if (gamepad2.dpad_up) {
                setRPM = farRPM;
            }
            else if(gamepad2.dpad_down){
                setRPM = closeRPM;
            }
            else if (gamepad2.touchpad) {
                setRPM = 0;
            }
            outtake.setRPM(setRPM);

            //Turret
            /*************************************Turret Auto-Aim**************************************/
            ElapsedTime turretClock = new ElapsedTime();
            turret.aimAtGoal(
                    currentPose.getX(),
                    currentPose.getY(),
                    Math.toDegrees(currentPose.getHeading())
            );
            double turretTime = turretClock.milliseconds();
            /*****************************************************************************************/

            /*if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0){
                turret.setPower(gamepad2.right_trigger);
            }
            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger > 0){
                turret.setPower(-gamepad2.left_trigger);
            }

            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0){
                turret.setPower(0);
            }*/



            if (kickstandButton.toggle(gamepad1.share)){
                kickstand.updatePos(KickstandServo.KickstandServoConfig.up);
            }
            else{
                kickstand.setPower(0);
            }


            //Telemetry

            multiTelemetry.addLine("==========================================");
            multiTelemetry.addData("Loop Time", loopTime.milliseconds());
            multiTelemetry.addData("Spindex Updater Loop Time", spindex.getThreadLoopTime());
            multiTelemetry.addLine("------------------------------------------");
            multiTelemetry.addData("Spindex Index", spindex.getIndex());
            multiTelemetry.addData("Slot Status", spindex.getSlotStatus()[0] + " " + spindex.getSlotStatus()[1] + " " + spindex.getSlotStatus()[2]);
            multiTelemetry.addData("Color", colorSensor.getHue());
            multiTelemetry.addData("At Target?", spindex.atTarget());
            multiTelemetry.addData("Spindex Power", spindex.getPower());
            multiTelemetry.addData("Automated Loading", spindex.isAutoLoading());
            multiTelemetry.addData("Outtaking?", spindex.isOuttakeing());
            multiTelemetry.addData("Kickstand Pos", kickstand.getPosition());
            multiTelemetry.addData("Kickstand Voltage", kickstand.getVoltage());
            multiTelemetry.addLine("------------------------------------------");
            multiTelemetry.addData("Distance", colorSensor.getDistance());
            multiTelemetry.addData("Right Pod", backRightDrive.getCurrentPosition());
            multiTelemetry.addData("Left Pod", backLeftDrive.getCurrentPosition());
            multiTelemetry.addData("Strafe Pod", frontRightDrive.getCurrentPosition());
            multiTelemetry.addLine("==========================================");
            multiTelemetry.addLine("Spindex Mode: " + (spindex.isOuttakeing() ? "Outtake" : "Intake"));
            multiTelemetry.addLine("Spindex Voltage: " + spindex.getVoltage());
            multiTelemetry.addLine("Spindex Angular Pos: " + AngleUnit.normalizeDegrees(spindex.getPos()));
            multiTelemetry.addLine("Spindex Angular Pos Rel: " + spindex.getEncPos());
            multiTelemetry.addLine("Turret Cycle Time: " + turretTime);
            //multiTelemetry.addLine("Colors: " + Arrays.toString(huskyLens.getColors()));
            multiTelemetry.update();
        }
    }
}