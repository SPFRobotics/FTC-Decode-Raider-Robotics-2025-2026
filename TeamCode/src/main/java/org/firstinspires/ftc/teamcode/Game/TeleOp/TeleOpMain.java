package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickstandServo;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.UpdateSpindex;
import org.firstinspires.ftc.teamcode.Resources.Button;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake.OuttakeConfig.*;

@TeleOp(name="Tele-Op Main")
public class TeleOpMain extends LinearOpMode {
    private Intake intake = null;
    private Outtake outtake = null;
    private KickerSpindex kicker = null;

    //Multiplys the motor power by a certain amount to lower or raise the speed of the motors
    private double speedFactor =  1;

    //Buttons
    private Button spindexModeToggle = new Button();
    private Button spindexRightBumper = new Button();
    private Button spindexLeftBumper = new Button();

    private Button intakeButton = new Button();
    private Button autoLoad = new Button();
    private double setRPM = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

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

        //Set auto load and launch to true as default
        autoLoad.changeState(true);

        //Initialize Telemetry
        waitForStart();
        telemetry.setMsTransmissionInterval(16);
        if (opModeIsActive()){
            updateSpindex.start();
        }

        ElapsedTime loopTime = new ElapsedTime();
        while (opModeIsActive()) {
            loopTime.reset();

            /*************************************Drive Train Control**************************************/
            //Allows speed to be halved
            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
                speedFactor = 0.5;
            }
            else{
                speedFactor = 1;
            }

            double y = -gamepad1.left_stick_y * speedFactor; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * speedFactor;
            double rx = gamepad1.right_stick_x * speedFactor;

            frontLeftDrive.setPower(y + x + rx);
            backLeftDrive.setPower(y - x + rx);
            frontRightDrive.setPower(y - x - rx);
            backRightDrive.setPower(y + x - rx);
            backRightDrive.setPower(y + x - rx);
            /**********************************************************************************************/

            /*****************************Intake System************************************/
            boolean intakeActive = intakeButton.toggle(gamepad1.right_bumper);
            if (intakeActive && !gamepad1.left_bumper) {
                intake.setPower(1);
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
                autoLoad.changeState(false);
                spindex.addIndex();
            }
            if (spindexLeftBumper.press(gamepad2.left_bumper)) {
                autoLoad.changeState(false);
                spindex.subtractIndex();
            }
            //Sets either intake or outtake mode
            spindex.setMode(spindexModeToggle.toggle(gamepad2.circle));
            /************************************************************/

            /*********************Kicker and index emptying logic**********************/
            kicker.automate(gamepad2.crossWasPressed() && spindex.isOuttakeing());
            if (gamepad2.crossWasPressed() && outtake.getPower() != 0){
                spindex.clearBall(spindex.getIndex());
            }
            /**************************************************************************/

            spindex.setAutoLoadMode(autoLoad.toggle(gamepad2.triangle) && !spindex.isOuttakeing());
            spindex.autoLoad(colorSensor);

            /*if (spindex.getMode()){
                spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], true);
            }
            else{
                spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], true);
            }*/

            //Controls gamepad rumble
            if (setRPM == closeRPM && outtake.getRPM() >= setRPM){
                gamepad2.rumble(100);
            }
            else if (setRPM == farRPM & outtake.getRPM() >= setRPM){
                gamepad2.rumble(100);
            }
            else{
                gamepad2.stopRumble();
            }

            // Outtake control - right trigger
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

            // Driver Hub
            telemetry.addLine("==========================================");
            telemetry.addData("Loop Time", loopTime.milliseconds());
            telemetry.addData("Spindex Updater Loop Time", spindex.getThreadLoopTime());
            telemetry.addLine("------------------------------------------");
            telemetry.addData("Spindex Index", spindex.getIndex());
            telemetry.addData("Slot Status", spindex.getSlotStatus()[0] + " " + spindex.getSlotStatus()[1] + " " + spindex.getSlotStatus()[2]);
            telemetry.addData("Automated Loading", spindex.isAutoLoading());
            telemetry.addData("Outtaking?", spindex.isOuttakeing());
            telemetry.addLine("------------------------------------------");
            telemetry.addData("Distance", colorSensor.getDistance());
            telemetry.addLine("==========================================");
            telemetry.update();
        }
        //Tells spindex thread to end execution
        spindex.exitProgram();
    }
}