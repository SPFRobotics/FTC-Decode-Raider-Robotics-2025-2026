package org.firstinspires.ftc.teamcode.Game;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.closeRPM;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.farRPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assets.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.DualColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.Assets.Button;

import java.util.List;

//@TeleOp(name="Tele-Op Red")
public class TeleRed extends LinearOpMode {
    Intake intake = null;
    ElapsedTime loopTime;
    Outtake outtake = null;
    KickerSpindex kicker = null;
    Chassis chassis = null;
    Turret turret = null;
    DualColorFetch colorSensor = null;
    Spindex spindex = null;
    LedLights leds = null;
    List<LynxModule> allHubs = null;

    Follower follower;
    Pose currentPose;

    Button spindexModeToggle = new Button();
    Button spindexRightBumper = new Button();
    Button spindexLeftBumper = new Button();
    Button intakeButton = new Button();
    Button autoLoad = new Button();
    Button turretToggle = new Button();
    Button holdPos = new Button();
    Limelight limelight;

    Button autoAimTurretButton = new Button();

    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashBoardTele = dash.getTelemetry();
    MultipleTelemetry multiTelemetry = new MultipleTelemetry();

    double speedFactor = 1;
    boolean fieldCentric = true;
    boolean missing = false;
    double setRPM = 0;
    int currentPos = 0;
    int kickerCount = 1;

    public void runOpMode(){
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        colorSensor = new DualColorFetch(hardwareMap);
        spindex = new Spindex(hardwareMap);
        chassis = new Chassis(hardwareMap);
        leds = new LedLights(hardwareMap);
        turret = new Turret(hardwareMap, false, limelight);
        turret.setTargetTag(24);
        if (PoseStorage.turretValid) {
            turret.setInitialAngle(PoseStorage.turretStartPos);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.poseEnd);
        Pose holdingPosition = null;

        follower.startTeleopDrive();
        outtake = new Outtake(hardwareMap, kicker);

        autoLoad.changeState(true);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            follower.update();
            currentPose = follower.getPose();

            /*************************************Drive Train Control**************************************/
            if (gamepad1.left_stick_button){
                speedFactor = 0.25;
            }
            else if (gamepad1.right_stick_button){
                speedFactor = 0.5;
            }
            else{
                speedFactor = 1;
            }

            if (!holdPos.toggle(gamepad1.touchpad)) {
                if (gamepad1.touchpadWasPressed()){
                    follower.startTeleopDrive();
                }
                follower.setTeleOpDrive(-gamepad1.left_stick_y * speedFactor, -gamepad1.left_stick_x * speedFactor, -gamepad1.right_stick_x * speedFactor, true);
            }
            else if (gamepad1.touchpadWasPressed()){
                holdingPosition = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
                follower.holdPoint(holdingPosition);
            }

            /**********************************************************************************************/

            /*****************************Intake System************************************/
            boolean intakeActive = intakeButton.toggle(gamepad1.right_bumper && !spindex.isOuttakeing());
            if (intakeActive && !gamepad1.left_bumper) {
                intake.intakeOn(true);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);

            }
            if (intake.getPower() > 0){
                gamepad2.rumble(100);
            }
            /******************************************************************************/

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

            spindex.setMode(spindexModeToggle.toggle(gamepad2.circle));

            spindex.setAutoLoadMode(autoLoad.toggle(gamepad2.share) && !spindex.isOuttakeing());
            spindex.autoLoad(colorSensor);

            if (spindex.isOuttakeing()) {
                char[] slotColors = spindex.getSlotColors();
                spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()]);

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
                spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()]);
                leds.setColor(LedLights.BLUE, false);
            }
            /****************************************************************************************************/

            /*********************Outtake Logic**********************/
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
                    turret.noEncoder();
                }
                turret.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*0.25);
            }
            else if (turretToggleState){
                turret.unlockTurret();
                turret.aimWithLimelight(limelight.getLatestResult());
            }
            else{
                if (gamepad1.shareWasPressed()){
                    turret.unlockTurret();
                    turret.useEncoder();
                }
                turret.periodic(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            }

            /*****************************************************************************************/
        }
    }
}
