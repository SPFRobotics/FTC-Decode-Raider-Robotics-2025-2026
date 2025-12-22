package org.firstinspires.ftc.teamcode.Testing;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Game.Subsystems.ColorFinder;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerGrav;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.Scroll;

import java.io.FileNotFoundException;
import java.io.PrintWriter;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    private Button circle = new Button();
    private Button square = new Button();
    private DigitalChannel button = null;
    private Button kickerButton = new Button();
    private int ballCount = 0;
    //private Limelight limelight;
    private Button option = new Button();
    private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");
    private Scroll daddyRyan = new Scroll("Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.");
    private PrintWriter pen = new PrintWriter("/sdcard/spindex.txt");

    ColorFinder colorSensor = null;
    private int rgb[];
    int hsv[];



    public Test() throws FileNotFoundException {
    }

    public void runOpMode(){
        //limelight = new Limelight(hardwareMap);
        Spindex spindex = new Spindex(hardwareMap, true);
        //KickerSpindex kicker = new KickerSpindex(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        colorSensor = new ColorFinder(hardwareMap);
        button = hardwareMap.get(DigitalChannel.class, "button");
        Button intakeButton = new Button();

        waitForStart();
        while (opModeIsActive()) {
            rgb = colorSensor.getColor();
            hsv = colorSensor.rgbToHSV(rgb[0], rgb[1], rgb[2]);
            if (intakeButton.toggle(gamepad1.right_bumper)){
                intake.intakeOn();
            }
            else {
                intake.intakeOff();
            }
            //limelight.start();

            if (colorSensor.getDistance() <= 3 && spindex.getPower() == 0 && ballCount < 3) {
                spindex.addIndex();
                ballCount++;
            }

            if (option.toggle(gamepad1.options)) {
                spindex.moveToPos(outtakePos[spindex.getIndex()]);
                telemetry.addData("Target", outtakePos[spindex.getIndex()]);
            } else {
                spindex.moveToPos(intakePos[spindex.getIndex()]);
                telemetry.addData("Target", intakePos[spindex.getIndex()]);
            }

            kicker.automate(kickerButton.press(gamepad1.a));

            telemetry.setMsTransmissionInterval(16);
            telemetry.addData("Spindex Pos", spindex.getPos());
            telemetry.addData("Pos", spindex.spindexMotor.getCurrentPosition());
            telemetry.addData("Distance", colorSensor.getDistance());
            telemetry.addData("Color:", hsv[0]);
            telemetry.addLine(bigThree.foward());
            telemetry.addLine(daddyRyan.foward());
            telemetry.addData("Button State", button.getState());
            telemetry.update();
            pen.write(spindex.getPos() + ":" + spindex.getError() + ":" + spindex.getPower() + "\n");

            /*LLResult llresult = limelight.getLatestResult();
            Pose3D botpose = null;
            if (llresult != null && llresult.isValid()) {
                botpose = llresult.getBotpose_MT2();
                telemetry.addData("Target X", llresult.getTx());
                telemetry.addData("Target Area", llresult.getTa());
            } else {
                telemetry.addData("Target", "No valid target");
            }
            if (botpose != null) {
                telemetry.addData("BotPose", botpose.toString());
            }
            telemetry.addData("Distance", limelight.getDistanceFromTag());
            telemetry.update();*/
        }
        pen.close();
    }
}
