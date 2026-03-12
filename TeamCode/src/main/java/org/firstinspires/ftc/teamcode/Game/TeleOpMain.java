package org.firstinspires.ftc.teamcode.Game;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.closeRPM;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.farRPM;

import com.acmerobotics.dashboard.FtcDashboard;
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

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.List;

@TeleOp(name="Tele-Op Main")
public class TeleOpMain extends LinearOpMode {
    //Hardware Devices
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
    Limelight limelight;

    Button autoLaunchButton = new Button();
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
    boolean autoLaunch = false;

    public void runOpMode(){
        // Initialize subsystems
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);
        kicker = new KickerSpindex(hardwareMap);
        colorSensor = new DualColorFetch(hardwareMap);
        spindex = new Spindex(hardwareMap);
        chassis = new Chassis(hardwareMap);
        leds = new LedLights(hardwareMap);
        //ZucskyLens huskyLens = new ZucskyLens(hardwareMap);
        turret = new Turret(hardwareMap, PoseStorage.blueAlliance, limelight);
        PrintWriter pen = null;
        try{
            pen = new PrintWriter("/sdcard/turret.txt", "ASCII");
        }
        catch(FileNotFoundException e){

        }
        catch (UnsupportedEncodingException e){

        }

        //Pedro Pathing for turret
        follower = Constants.createFollower(hardwareMap);
        //Pose pose = new Pose(72, 72, Math.toRadians(45));
        //follower.setStartingPose(pose);
        follower.setStartingPose(PoseStorage.poseEnd);
        Pose parkingPose = new Pose(39, 33, 180);


        follower.startTeleopDrive();
        outtake = new Outtake(hardwareMap, kicker);

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
        ElapsedTime runTime = new ElapsedTime();
        chassis.setBrakeMode();
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
            if (gamepad1.right_bumper){
                speedFactor = 0.25;
            }
            else if (gamepad1.left_bumper){
                speedFactor = 0.1;
            }
            else{
                speedFactor = 1;
            }


            double y = -gamepad1.left_stick_y * speedFactor; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * speedFactor;
            double rx = gamepad1.right_stick_x * speedFactor;

            // When following a path, let the follower control the drive; chassis would overwrite and cause shuddering
            if (!follower.isBusy()) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y * speedFactor, -gamepad1.left_stick_x * speedFactor, -gamepad1.right_stick_x * speedFactor, true); // Remember, Y stick is reversed!
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

            //Auto Launch: update toggle first so all controls below see the current state
            autoLaunch = autoLaunchButton.toggle((gamepad2.right_trigger>=0.3));
            spindex.setAutoLaunchMode(autoLaunch);
            if (autoLaunch) {
                spindex.autoLaunch(kicker);
                if (!spindex.isAutoLaunching()) {
                    autoLaunch = false;
                    autoLaunchButton.changeState(false);
                }
            } else {
                spindex.resetAutoLaunch();
            }

            /*********************Kicker and index emptying logic**********************/
            if (!autoLaunch) {
                boolean crossWasPressed = gamepad2.crossWasPressed();
                kicker.automate(crossWasPressed && spindex.isOuttakeing());
                if (crossWasPressed && spindex.isOuttakeing() && outtake.getPower() != 0) {
                    spindex.clearBall(spindex.getIndex());
                }
            }
            /**************************************************************************/

            /*******************************************Spindex Logic********************************************/
            if (spindexRightBumper.press(gamepad2.right_bumper) && !autoLaunch) {
                if (!spindex.isOuttakeing()) {
                    autoLoad.changeState(false);
                }
                spindex.addIndex();
                missing = false;
            }
            if (spindexLeftBumper.press(gamepad2.left_bumper) && !autoLaunch) {
                if (!spindex.isOuttakeing()) {
                    autoLoad.changeState(false);
                }
                spindex.subtractIndex();
                missing = false;
            }

            //Sets either intake or outtake mode
            spindex.setMode(autoLaunch || spindexModeToggle.toggle(gamepad2.circle));

            //Automatic Loading
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
            // Outtake control
            if (gamepad2.dpad_up) {
                setRPM = farRPM;
            } else if (gamepad2.dpad_down) {
                setRPM = closeRPM;
            } else if (gamepad2.touchpad) {
                setRPM = 0;
            }
            outtake.setRPM(setRPM);

            //Controls gamepad rumble
            double rumbleRange = Math.abs(setRPM - outtake.getRPM());
            if (setRPM == closeRPM && rumbleRange <= 100) {
                gamepad2.rumble(100);
            } else if (setRPM == farRPM & rumbleRange <= 100) {
                gamepad2.rumble(100);
            } else {
                gamepad2.stopRumble();
            }

            if (gamepad1.circleWasPressed() && !follower.isBusy()) {
                Path pathToTarget = new Path(new BezierLine(currentPose, parkingPose));
                pathToTarget.setLinearHeadingInterpolation(currentPose.getHeading(),parkingPose.getHeading());
                follower.followPath(pathToTarget);
            }

           // outtake.setPower(1);
            /********************************************************/

            /*************************************Turret Auto-Aim**************************************/
            //Vector velocity = follower.getVelocity();
            /*
            if (turretToggle.toggle(gamepad1.share) && gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0){
                if (gamepad1.shareWasPressed()){
                    turret.noEncoder();
                }
                turret.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*0.25);
            }
            else if (turretToggle.toggle(gamepad1.share)){
                turret.aimWithLimelight(limelight.getLatestResult());
            }
            else{
                if (gamepad1.shareWasPressed()){
                    turret.useEncoder();
                }
                turret.periodic(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            }
*/
            /*****************************************************************************************/

            //Telemetry

            multiTelemetry.addLine("==========================================");
            //spindex.showTelemetry(multiTelemetry);
            //colorSensor.showTelemetry(multiTelemetry);
            turret.showTelemetry(multiTelemetry);
            //colorSensor.showTelemetry(telemetry);
            multiTelemetry.addData("Outtake RPM", outtake.getRPM());
            multiTelemetry.addLine("==========================================");

            multiTelemetry.addData("Loop Time", loopTime.milliseconds());
            multiTelemetry.update();
            //turret.log(pen);
        }
        //pen.close();
    }
}