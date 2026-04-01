package org.firstinspires.ftc.teamcode.Game;
import static org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextOuttake.closeRPM;
import static org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextOuttake.farRPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assets.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Assets.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.DualColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextIntake;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextOuttake;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextTurret;
import org.firstinspires.ftc.teamcode.Subsystems.OldSubsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Assets.Button;

import java.util.List;
@Disabled
@TeleOp(name="Tele-Op Main")
public class TeleOpMain extends LinearOpMode {
    //Hardware Devices
    NextIntake intake = NextIntake.INSTANCE;
    ElapsedTime loopTime;
    NextOuttake outtake = NextOuttake.INSTANCE;
    KickerSpindex kicker = null;
    Chassis chassis = null;
    NextTurret turret = NextTurret.INSTANCE;
    DualColorFetch colorSensor = null;
    NextSpindex spindex = NextSpindex.INSTANCE;
    LedLights leds = null;
    List<LynxModule> allHubs = null;


    //Pedro Pathing
    Follower follower;
    Pose currentPose;

    //Buttons
    Button spindexModeToggle = new Button();
    Button spindexRightBumper = new Button();
    Button spindexLeftBumper = new Button();
    Button intakeButton = new Button();
    Button autoLoad = new Button();
    Button turretToggle = new Button();
    Button holdPos = new Button();
    Limelight limelight;

    //Button autoLaunchButton = new Button();
    Button autoAimTurretButton = new Button();

    //Telemetry
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashBoardTele = dash.getTelemetry();
    MultipleTelemetry multiTelemetry = new MultipleTelemetry();

    //Other Varibles
    //Multiplys the motor power by a certain amount to lower or raise the speed of the motor
    double speedFactor = 1;
    boolean fieldCentric = true;
    boolean missing = false;
    double setRPM = 0;
    int currentPos = 0;
    int kickerCount = 1;
    //boolean autoLaunch = false;

    public void runOpMode(){
        // Initialize subsystems
        intake.initialize();
        limelight = new Limelight(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        colorSensor = new DualColorFetch(hardwareMap);
        spindex.initialize();
        chassis = new Chassis(hardwareMap);
        leds = new LedLights(hardwareMap);
        //ZucskyLens huskyLens = new ZucskyLens(hardwareMap);
        turret.setGoalCoords(PoseStorage.blueAlliance);
        turret.setLimelight(limelight);
        turret.initialize();
        if (PoseStorage.turretValid) {
            turret.setInitialAngle(PoseStorage.turretStartPos);
        }

        //Pedro Pathing for turret
        follower = Constants.createFollower(hardwareMap);
        //Pose pose = new Pose(72, 72, Math.toRadians(45));
        //follower.setStartingPose(pose);
        follower.setStartingPose(PoseStorage.poseEnd);
        //follower.setStartingPose(PoseStorage.teleop);
        Pose holdingPosition = null;


        follower.startTeleopDrive();
        outtake.setKicker(kicker);
        outtake.initialize();

        //Set autoload and launch to true as default
        autoLoad.changeState(true);

        //Caching using LynxModule
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Telemetry
        multiTelemetry.addTelemetry(telemetry);
        multiTelemetry.addTelemetry(dashBoardTele);
        multiTelemetry.setMsTransmissionInterval(16);
        loopTime = new ElapsedTime();

        while (opModeInInit()){
            follower.update();
            leds.cycleColors(10);
        }

        turret.setAlignmentEnabled(true);
        waitForStart();
        chassis.setBrakeMode();
        ElapsedTime runTime = new ElapsedTime();
        while (opModeIsActive()){
            loopTime.reset();
            //Reset Cache for Lynx Modules
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            //Update robot location (Pedro Pathing)
            follower.update();
            currentPose = follower.getPose();

            /*************************************Drive Train Control**************************************/
            //Using Pedro Pathing for Tele-Op drive
            //Allows speed to be halved
            if (gamepad1.left_stick_button){
                speedFactor = 0.25;
            }
            else if (gamepad1.right_stick_button){
                speedFactor = 0.5;
            }
            else{
                speedFactor = 1;
            }


            /*double y = -gamepad1.left_stick_y * speedFactor; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * speedFactor;
            double rx = gamepad1.right_stick_x * speedFactor;*/

            // When following a path, let the follower control the drive; chassis would overwrite and cause shuddering
            if (!holdPos.toggle(gamepad1.touchpad)) {
                if (gamepad1.touchpadWasPressed()){
                    follower.startTeleopDrive();
                }
                follower.setTeleOpDrive(-gamepad1.left_stick_y * speedFactor, -gamepad1.left_stick_x * speedFactor, -gamepad1.right_stick_x * speedFactor, true); // Remember, Y stick is reversed!
            }
            else if (gamepad1.touchpadWasPressed()){
                holdingPosition = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
                follower.holdPoint(holdingPosition);
            }

            /**********************************************************************************************/

            /*****************************Intake System************************************/
            boolean intakeActive = intakeButton.toggle(gamepad1.right_bumper && !spindex.isOuttakeing());
            if (intakeActive && !gamepad1.left_bumper) {
                intake.turnOn();
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.turnOff();

            }
            intake.periodic();
            if (intake.getMotorPower() > 0){
                gamepad2.rumble(100);
            }
            /******************************************************************************/

            //Auto Launch: update toggle first so all controls below see the current state
            /*autoLaunch = autoLaunchButton.toggle((gamepad2.right_trigger>=0.3));
            spindex.setAutoLaunchMode(autoLaunch);
            if (autoLaunch) {
                spindex.autoLaunch(kicker);
                if (!spindex.isAutoLaunching()) {
                    autoLaunch = false;
                    autoLaunchButton.changeState(false);
                }
            } else {
                spindex.resetAutoLaunch();
            }*/

            /*********************Kicker and index emptying logic**********************/
            boolean crossWasPressed = gamepad2.crossWasPressed();
            kicker.automate(crossWasPressed && spindex.isOuttakeing());
            if (crossWasPressed && spindex.isOuttakeing() && outtake.getPower() != 0) {
                spindex.clearBall(spindex.getIndex());
            }
            /**************************************************************************/

            /*******************************************Spindex Logic********************************************/
            if (spindexRightBumper.press(gamepad2.right_bumper)) {
                if (!spindex.isOuttakeing()) {
                    autoLoad.changeState(false);
                }
                spindex.addIndex();
                missing = false;
            }
            if (spindexLeftBumper.press(gamepad2.left_bumper)) {
                if (!spindex.isOuttakeing()) {
                    autoLoad.changeState(false);
                }
                spindex.subtractIndex();
                missing = false;
            }

            //Sets either intake or outtake mode
            spindex.setMode(spindexModeToggle.toggle(gamepad2.circle));

            //Automatic Loading
            spindex.setAutoLoadMode(autoLoad.toggle(gamepad2.share) && !spindex.isOuttakeing());
            spindex.autoLoad(colorSensor);

            if (spindex.isOuttakeing()) {
                char[] slotColors = spindex.getSlotColors();
                spindex.moveToPos(NextSpindex.outtakePos[spindex.getIndex()]);

                if (slotColors[spindex.getIndex()] == 'E' || missing){
                    leds.setColor(LedLights.RED, false);
                }
                else if (slotColors[spindex.getIndex()] == 'G'){
                    leds.setColor(LedLights.GREEN, false);
                }
                else if (slotColors[spindex.getIndex()] == 'P'){
                    leds.setColor(LedLights.INDIGO, false);
                }
                if (gamepad2.squareWasPressed()){
                    int indexOfColor = spindex.getIndexOfColor('P');
                    missing = indexOfColor < 0;
                    spindex.setIndex(indexOfColor < 0 ? spindex.getIndex() : indexOfColor);
                }
                else if (gamepad2.triangleWasPressed()){
                    int indexOfColor = spindex.getIndexOfColor('G');
                    missing = indexOfColor < 0;
                    spindex.setIndex(indexOfColor < 0 ? spindex.getIndex() : indexOfColor);
                }
            }
            else {
                spindex.moveToPos(NextSpindex.intakePos[spindex.getIndex()]);
                leds.setColor(LedLights.BLUE, false);
            }
            spindex.periodic();
            /****************************************************************************************************/

            /*********************Outtake Logic**********************/
            // Outtake control
            if (gamepad2.dpad_up) {
                setRPM = farRPM;
                turret.setShortMode(false);
            } else if (gamepad2.dpad_down) {
                setRPM = closeRPM;
                turret.setShortMode(true);
            } else if (gamepad2.touchpad) {
                setRPM = 0;
            }
            outtake.setRPM(setRPM);
            outtake.periodic();

            //Controls gamepad rumble
            double rumbleRange = Math.abs(setRPM - outtake.getRPM());
            if (setRPM == closeRPM && rumbleRange <= 100) {
                gamepad2.rumble(100);
            } else if (setRPM == farRPM & rumbleRange <= 100) {
                gamepad2.rumble(100);
            } else {
                gamepad2.stopRumble();
            }

            /********************************************************/

            /*************************************Turret Auto-Aim**************************************/
            boolean turretToggleState = turretToggle.toggle(gamepad1.share);
            if (turretToggleState && (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0)){
                if (gamepad1.shareWasPressed()){
                    turret.unlockTurret();
                }
            }
            else if (turretToggleState){
                turret.unlockTurret();
            }
            else{
                if (gamepad1.shareWasPressed()){
                    turret.unlockTurret();
                }
            }
            turret.update(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            turret.periodic();

            /*****************************************************************************************/

            //Telemetry

            /*multiTelemetry.addLine("==========================================");
            //spindex.showTelemetry(multiTelemetry);
            //colorSensor.showTelemetry(multiTelemetry);
            turret.showTelemetry(multiTelemetry);
            //colorSensor.showTelemetry(telemetry);
            multiTelemetry.addData("Outtake RPM", outtake.getRPM());
            multiTelemetry.addLine("==========================================");

            //turret.log(pen);*/
            //pen.write(runTime.milliseconds() + ":" + colorSensor.getDistances()[0] + "\n");
            //pen1.write(runTime.milliseconds() + ":" + colorSensor.getDistances()[1] + "\n");
            //multiTelemetry.addData("Loop Time", loopTime.milliseconds());
            //multiTelemetry.update();
        }
        //pen.close();
    }
}