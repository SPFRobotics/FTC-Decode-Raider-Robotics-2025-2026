package org.firstinspires.ftc.teamcode.Testing;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerGrav;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.Scroll;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    private Button circle = new Button();
    private Button square = new Button();
    //private Limelight limelight;
    private Button option = new Button();
    private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");
    private Scroll daddyRyan = new Scroll("Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.");
    public void runOpMode(){
        //limelight = new Limelight(hardwareMap);
        Spindex spindex = new Spindex(hardwareMap, true);
        //KickerSpindex kicker = new KickerSpindex(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                intake.intakeOn();
            }
            else {
                intake.intakeOff();
            }
            //limelight.start();


            if (circle.press(gamepad1.circle)) {
                spindex.addIndex();
            } else if (square.press(gamepad1.square)) {
                spindex.subtractIndex();
            }
            if (option.toggle(gamepad1.options)) {
                spindex.moveToPos(outtakePos[spindex.getIndex()]);
                telemetry.addData("Target", outtakePos[spindex.getIndex()]);
            } else {
                spindex.moveToPos(intakePos[spindex.getIndex()]);
                telemetry.addData("Target", intakePos[spindex.getIndex()]);
            }
            telemetry.setMsTransmissionInterval(16);
            telemetry.addData("Spindex Pos", spindex.getPos());
            telemetry.addData("Pos", spindex.spindexMotor.getCurrentPosition());
            telemetry.addLine(bigThree.foward());
            telemetry.addLine(daddyRyan.foward());
            telemetry.update();



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
    }
}
