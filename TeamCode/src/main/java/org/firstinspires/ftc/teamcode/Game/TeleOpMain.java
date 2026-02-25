package org.firstinspires.ftc.teamcode.Game;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.closeRPM;
import static org.firstinspires.ftc.teamcode.Subsystems.Outtake.OuttakeConfig.farRPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.LedLights;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Resources.Button;

import java.util.List;

@TeleOp(name="Tele-Op Main")
public class TeleOpMain extends OpMode {
    //Hardware Devices
    Intake intake = null;
    Outtake outtake = null;
    KickerSpindex kicker = null;
    Turret turret = null;
    ColorFetch colorSensor = null;
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
    Button autoAimTurretButton = new Button();

    //Telemetry
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashBoardTele = dash.getTelemetry();
    MultipleTelemetry multiTelemetry = new MultipleTelemetry();

    //Other Varibles
    //Multiplys the motor power by a certain amount to lower or raise the speed of the motor
    double speedFactor = 1;
    boolean fieldCentric = true;
    double setRPM = 0;

    public void init() {
        // Initialize subsystems
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, true);
        kicker = new KickerSpindex(hardwareMap);
        colorSensor = new ColorFetch(hardwareMap);
        spindex = new Spindex(hardwareMap);
        leds = new LedLights(hardwareMap);
        //HuskyLensController huskyLens = new HuskyLensController(hardwareMap);
        turret = new Turret(hardwareMap, true);

        //Pedro Pathing for turret
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.startTeleopDrive();

        //Set autoload and launch to true as default
        autoLoad.changeState(true);

        //Caching using LynxModule
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //Telemetry
        multiTelemetry.addTelemetry(telemetry);
        multiTelemetry.addTelemetry(dashBoardTele);
        multiTelemetry.setMsTransmissionInterval(16);
    }

    public void init_loop() {
        follower.update();
        leds.cycleColors(10);
    }

    public void loop() {
        ElapsedTime loopTime = new ElapsedTime();

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
        speedFactor = gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1 ? 0.5 : 1; //Ternary if statement (Condition ? This is true : This is false) will return a value based on the condition
        follower.setTeleOpDrive(-gamepad1.left_stick_y * speedFactor, gamepad1.left_stick_x * speedFactor, gamepad1.right_stick_x * speedFactor, fieldCentric); // Remember, Y stick is reversed!
        /**********************************************************************************************/

        /*****************************Intake System************************************/
        boolean intakeActive = intakeButton.toggle(gamepad1.right_bumper);
        if (intakeActive && !gamepad1.left_bumper) {
            intake.intakeOn(true);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);

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
        }
        if (spindexLeftBumper.press(gamepad2.left_bumper)) {
            if (!spindex.isOuttakeing()) {
                autoLoad.changeState(false);
            }
            spindex.subtractIndex();
        }

        //Sets either intake or outtake mode
        spindex.setMode(spindexModeToggle.toggle(gamepad2.circle));

        //Automatic Loading
        spindex.setAutoLoadMode(autoLoad.toggle(gamepad2.triangle) && !spindex.isOuttakeing());
        spindex.autoLoad(colorSensor);

        //Controls actual spindex movement
        if (spindex.isOuttakeing()) {
            spindex.moveToPos(Spindex.SpindexValues.outtakePos[spindex.getIndex()], 4);
            leds.setColor(leds.GREEN, false);
        } else {
            spindex.moveToPos(Spindex.SpindexValues.intakePos[spindex.getIndex()], 4);
            leds.setColor(leds.BLUE, false);
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
        if (setRPM == closeRPM && outtake.getRPM() >= setRPM) {
            gamepad2.rumble(100);
        } else if (setRPM == farRPM & outtake.getRPM() >= setRPM) {
            gamepad2.rumble(100);
        } else {
            gamepad2.stopRumble();
        }
        /********************************************************/

        /*************************************Turret Auto-Aim**************************************/
        turret.aimAtGoal(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        /*****************************************************************************************/

        //Telemetry
        multiTelemetry.addLine("==========================================");
        spindex.showTelemetry(multiTelemetry);
        colorSensor.showTelemetry(multiTelemetry);
        //turret.showTelemetry(multiTelemetry, currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        multiTelemetry.addLine("==========================================");
        multiTelemetry.update();
    }
}