package org.firstinspires.ftc.teamcode.Game.TeleOp;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.closeRPM;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.farRPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.KickstandServo;
import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.UpdateSpindex;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.PedroPathing.Constants;

@TeleOp(name="Tele-Op Pedro")
public class PedroTeleOP extends OpMode {
    private Follower follower;
    private Intake intake = null;
    private Outtake outtake = null;
    private KickerSpindex kicker = null;
    private Turret turret = null;
    private ColorFetch colorSensor = null;
    private Spindex spindex = null;
    private UpdateSpindex updateSpindex = null;
    private KickstandServo kickstand = null;
    private LedLights leds = null;

    private double speedFactor = 1;
    private boolean fieldCentric = true;

    private Button spindexModeToggle = new Button();
    private Button spindexRightBumper = new Button();
    private Button spindexLeftBumper = new Button();
    private Button kickstandButton = new Button();
    private Button fieldholdButton = new Button();

    private Button intakeButton = new Button();
    private Button autoLoad = new Button();

    private double setRPM = 0;
    ElapsedTime intakeReverseTimer = new ElapsedTime();
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashBoardTele = dash.getTelemetry();
    MultipleTelemetry multiTelemetry = new MultipleTelemetry();

    private Pose currentPose;
    private ElapsedTime loopTime;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.update();

        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, true);
        kicker = new KickerSpindex(hardwareMap);
        colorSensor = new ColorFetch(hardwareMap);
        spindex = new Spindex(hardwareMap);
        updateSpindex = new UpdateSpindex(spindex);
        kickstand = new KickstandServo(hardwareMap);
        leds = new LedLights(hardwareMap);
        turret = new Turret(hardwareMap, true);

        autoLoad.changeState(true);
    }

    @Override
    public void init_loop() {
        leds.cycleColors(10);
        follower.update();
    }

    @Override
    public void start() {
        currentPose = follower.getPose();
        follower.startTeleopDrive();

        loopTime = new ElapsedTime();
        multiTelemetry.addTelemetry(telemetry);
        multiTelemetry.addTelemetry(dashBoardTele);
        multiTelemetry.setMsTransmissionInterval(16);
    }

    @Override
    public void loop() {
        loopTime.reset();
        follower.update();

        /*************************************Drive Train Control**************************************/
        if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
            speedFactor = 0.5;
        }
        else{
            speedFactor = 1;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedFactor,
                gamepad1.left_stick_x * speedFactor,
                gamepad1.right_stick_x * speedFactor,
                fieldCentric
        );

        boolean fieldHold = fieldholdButton.toggle(gamepad2.left_stick_button);

        if (fieldHold){
            follower.holdPoint(currentPose);
        }

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

        if (setRPM == closeRPM && outtake.getRPM() >= setRPM){
            gamepad2.rumble(100);
        }
        else if (setRPM == farRPM & outtake.getRPM() >= setRPM){
            gamepad2.rumble(100);
        }
        else{
            gamepad2.stopRumble();
        }

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

        if (kickstandButton.toggle(gamepad1.share)){
            kickstand.updatePos(KickstandServo.KickstandServoConfig.up);
        }
        else{
            kickstand.setPower(0);
        }

        /*************************************Turret Auto-Aim**************************************/
        turret.aimAtGoal(
                currentPose.getX(),
                currentPose.getY(),
                Math.toDegrees(currentPose.getHeading())
        );
        /*****************************************************************************************/

        multiTelemetry.addLine("==========================================");
        multiTelemetry.addData("Loop Time", loopTime.milliseconds());
        multiTelemetry.addData("Spindex Updater Loop Time", spindex.getThreadLoopTime());
        multiTelemetry.addLine("------------------------------------------");
        multiTelemetry.addData("Spindex Index", spindex.getIndex());
        multiTelemetry.addData("Slot Status", spindex.getSlotStatus()[0] + " " + spindex.getSlotStatus()[1] + " " + spindex.getSlotStatus()[2]);
        multiTelemetry.addData("Color", colorSensor.getHue());
        multiTelemetry.addData("Spindex At Target?", spindex.atTarget());
        multiTelemetry.addData("Spindex Power", spindex.getPower());
        multiTelemetry.addData("Automated Loading", spindex.isAutoLoading());
        multiTelemetry.addData("Outtaking?", spindex.isOuttakeing());
        multiTelemetry.addData("Kickstand Pos", kickstand.getPosition());
        multiTelemetry.addData("Kickstand Voltage", kickstand.getVoltage());
        multiTelemetry.addLine("------------------------------------------");
        multiTelemetry.addData("Distance", colorSensor.getDistance());
        multiTelemetry.addData("Position", follower.getPose().toString());
        multiTelemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        multiTelemetry.addData("Field Centric", fieldCentric);
        multiTelemetry.addData("Turret At Target", turret.isTurretAtTarget());
        multiTelemetry.addLine("==========================================");
        multiTelemetry.addLine("Spindex Mode: " + (spindex.isOuttakeing() ? "Outtake" : "Intake"));
        multiTelemetry.addLine("Spindex Voltage: " + spindex.getVoltage());
        multiTelemetry.addLine("Spindex Angular Pos: " + AngleUnit.normalizeDegrees(spindex.getPos()));
        multiTelemetry.addLine("Spindex Angular Pos Rel: " + spindex.getEncPos());
        multiTelemetry.update();
    }
}