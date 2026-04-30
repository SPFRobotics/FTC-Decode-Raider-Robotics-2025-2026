package org.firstinspires.ftc.teamcode.Game;
import static org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextOuttake.closeRPM;
import static org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextOuttake.farRPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.skeletonarmy.marrow.zones.PolygonZone;

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

@TeleOp(name="Tele-Op Red")
public class TeleRed extends LinearOpMode {
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

    //Shooting Zones (Marrow)
    final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(71, 23), new Point(93, 0));
    final PolygonZone closeLaunchZone = new PolygonZone(new Point(0, 144), new Point(71, 71), new Point(144, 144));
    final PolygonZone robotZone = new PolygonZone(18, 18);

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
        intake.initialize();
        limelight = new Limelight(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        colorSensor = new DualColorFetch(hardwareMap);
        spindex.initialize();
        chassis = new Chassis(hardwareMap);
        leds = new LedLights(hardwareMap);
        turret.setGoalCoords(false);
        turret.setLimelight(limelight);
        turret.initialize();
        turret.setTargetTag(24);
        if (PoseStorage.turretValid) {
            turret.setInitialAngle(PoseStorage.turretStartPos);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.poseEnd);
        Pose holdingPosition = null;

        follower.startTeleopDrive();
        outtake.setKicker(kicker);
        outtake.initialize();

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

            //Sync robot zone with current pose for shooting zone checks
            robotZone.setPosition(currentPose.getX(), currentPose.getY());
            robotZone.setRotation(currentPose.getHeading());

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
                follower.setTeleOpDrive(gamepad1.left_stick_y * speedFactor, -gamepad1.left_stick_x * speedFactor, -gamepad1.right_stick_x * speedFactor, true);
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

            /*********************Kicker and index emptying logic**********************/
            boolean inShootingZone = robotZone.isInside(farLaunchZone) || robotZone.isInside(closeLaunchZone);
            boolean crossWasPressed = gamepad2.crossWasPressed();
            kicker.automate(crossWasPressed && spindex.isOuttakeing() && inShootingZone);
            if (crossWasPressed && spindex.isOuttakeing() && outtake.getPower() != 0 && inShootingZone) {
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
        }
    }
}
