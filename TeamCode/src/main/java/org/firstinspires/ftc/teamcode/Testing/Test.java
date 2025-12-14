package org.firstinspires.ftc.teamcode.Testing;
import static org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex.SpindexValues.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerGrav;
import org.firstinspires.ftc.teamcode.Game.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.Scroll;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    private Button circle = new Button();
    private Button square = new Button();
    private Button option = new Button();
    private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");
    private Scroll daddyRyan = new Scroll("Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.");
    public void runOpMode(){
        Spindex spindex = new Spindex(hardwareMap, true);
        KickerSpindex kicker = new KickerSpindex(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (circle.press(gamepad1.circle)){
                spindex.addIndex();
            }
            else if (square.press(gamepad1.square)){
                spindex.subtractIndex();
            }
            if (option.toggle(gamepad1.options)){
                spindex.moveToPos(outtakePos[spindex.getIndex()]);
                telemetry.addData("Target", outtakePos[spindex.getIndex()]);
            }
            else{
                spindex.moveToPos(intakePos[spindex.getIndex()]);
                telemetry.addData("Target", intakePos[spindex.getIndex()]);
            }
            telemetry.setMsTransmissionInterval(16);
            telemetry.addData("Spindex Pos", spindex.getPos());
            telemetry.addLine(bigThree.foward());
            telemetry.addLine(daddyRyan.foward());
            telemetry.update();
        }
    }
}
