package org.firstinspires.ftc.teamcode.Testing;
import static org.firstinspires.ftc.teamcode.Testing.Test.Testing.intakePos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;
import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.Scroll;

@TeleOp(name="Test")
//@Disabled
public class Test extends LinearOpMode {
    @Config
    public static class Testing{
        public static int[] PIDF = {10, 3, 0, 0};
        public static double[] intakePos = {0, 120, 240};
    }
    private int index = 0;
    private Button moveSpindex = new Button();
    private Scroll bigThree = new Scroll("THE BIG 3 - Manav Shah - Ryan Zuck - Om Ram - Bassicly ryan is our dad, hes the founder, im the first born, om is second born. Om is like disregarded sometimes but its ok cuz hes a lovley boy and we all love om ramanathan");
    private Scroll daddyRyan = new Scroll("Ryan is our father. He will forever maintain us, sustain us, and push us forward towards victory. Ryan will save us. Ryan is Jewses.");
    public void runOpMode(){
        Spindex spindex = new Spindex(hardwareMap, true);
        waitForStart();
        while (opModeIsActive()){
            if (moveSpindex.press(gamepad1.a)){
                index++;
            }
            spindex.moveToPos(intakePos[Math.floorMod(index, 3)]);
            telemetry.getMsTransmissionInterval();
            telemetry.addData("Spindex Pos", spindex.getPos());
            telemetry.addData("Target", intakePos[Math.floorMod(index, 3)]);
            telemetry.addLine(bigThree.foward());
            telemetry.addLine(daddyRyan.foward());
            telemetry.update();
        }
    }
}
