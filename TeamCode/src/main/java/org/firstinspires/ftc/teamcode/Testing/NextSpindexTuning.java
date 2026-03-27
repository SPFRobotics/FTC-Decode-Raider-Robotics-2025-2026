package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.NextSpindex;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "NextSpindex Tuning", group = "Testing")
public class NextSpindexTuning extends LinearOpMode {

    private final double[] anglePresets = {0, 60, 120, 180, 240, 300};
    private int presetIndex = 0;
    private double targetAngle = 0;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevBack = false;
    private boolean prevRB = false;
    private boolean prevLB = false;

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        NextSpindex spindex = NextSpindex.INSTANCE;
        spindex.initialize();

        telemetry.addLine("=== NextSpindex PID Tuner ===");
        telemetry.addLine("Adjust kP/kI/kD in Dashboard under NextSpindex");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD UP/DOWN  : Cycle angle presets (0-300 by 60)");
        telemetry.addLine("  A             : Go to intake pos [current index]");
        telemetry.addLine("  B             : Go to outtake pos [current index]");
        telemetry.addLine("  X             : Go to 0 degrees");
        telemetry.addLine("  Y             : Go to 180 degrees");
        telemetry.addLine("  RB / LB       : Next / Prev slot index");
        telemetry.addLine("  BACK          : Re-apply PID from Dashboard");
        telemetry.addLine("  RIGHT STICK Y : Fine-tune angle (+/- 5 deg)");
        telemetry.update();

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double loopTime = loopTimer.milliseconds();
            loopTimer.reset();

            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            // --- Rebuild PID with latest Dashboard values ---
            if (gamepad1.back && !prevBack) {
                spindex.initialize();
                spindex.moveToPos(targetAngle);
            }
            prevBack = gamepad1.back;

            // --- Preset cycling ---
            if (gamepad1.dpad_up && !prevDpadUp) {
                presetIndex = Math.min(presetIndex + 1, anglePresets.length - 1);
                targetAngle = anglePresets[presetIndex];
                spindex.moveToPos(targetAngle);
            }
            prevDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !prevDpadDown) {
                presetIndex = Math.max(presetIndex - 1, 0);
                targetAngle = anglePresets[presetIndex];
                spindex.moveToPos(targetAngle);
            }
            prevDpadDown = gamepad1.dpad_down;

            // --- Quick positions ---
            if (gamepad1.a && !prevA) {
                targetAngle = NextSpindex.intakePos[spindex.getIndex()];
                spindex.moveToPos(targetAngle);
            }
            prevA = gamepad1.a;

            if (gamepad1.b && !prevB) {
                targetAngle = NextSpindex.outtakePos[spindex.getIndex()];
                spindex.moveToPos(targetAngle);
            }
            prevB = gamepad1.b;

            if (gamepad1.x && !prevX) {
                targetAngle = 0;
                spindex.moveToPos(targetAngle);
            }
            prevX = gamepad1.x;

            if (gamepad1.y && !prevY) {
                targetAngle = 180;
                spindex.moveToPos(targetAngle);
            }
            prevY = gamepad1.y;

            // --- Index cycling ---
            if (gamepad1.right_bumper && !prevRB) {
                spindex.addIndex();
            }
            prevRB = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !prevLB) {
                spindex.subtractIndex();
            }
            prevLB = gamepad1.left_bumper;

            // --- Fine-tune with right stick ---
            if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                targetAngle = targetAngle - gamepad1.right_stick_y * 2;
                spindex.moveToPos(targetAngle);
            }

            // --- Run the control loop ---
            spindex.periodic();

            // --- Telemetry ---
            double normAngle = spindex.getNormAngPos();
            double absAngle = spindex.getPos();
            double spindexError = spindex.getError();
            double motorPower = spindex.getPower();

            telemetry.addData("Target Angle", "%.1f°", targetAngle);
            telemetry.addData("Norm Angle (enc)", "%.1f°", normAngle);
            telemetry.addData("Abs Encoder", "%.1f°", absAngle);
            telemetry.addData("Error (ticks)", "%.1f", spindexError);
            telemetry.addData("Motor Power", "%.3f", motorPower);
            telemetry.addData("At Target?", !spindex.isBusy());
            telemetry.addLine("");
            telemetry.addData("Slot Index", spindex.getIndex());
            telemetry.addData("Intake Pos", "%.0f°", NextSpindex.intakePos[spindex.getIndex()]);
            telemetry.addData("Outtake Pos", "%.0f°", NextSpindex.outtakePos[spindex.getIndex()]);
            telemetry.addData("Slot Colors", Arrays.toString(spindex.getSlotColors()));
            telemetry.addLine("");
            telemetry.addData("Preset", "[%d] = %.0f°", presetIndex, anglePresets[presetIndex]);
            telemetry.addLine("");
            telemetry.addData("kP", NextSpindex.kP);
            telemetry.addData("kI", NextSpindex.kI);
            telemetry.addData("kD", NextSpindex.kD);
            telemetry.addData("kS (static FF)", NextSpindex.kS);
            telemetry.addData("kV (vel FF)", NextSpindex.kV);
            telemetry.addData("Tolerance", NextSpindex.positionToleranceTicks);
            telemetry.addLine("");
            telemetry.addData("Current (A)", "%.2f", spindex.getAmps());
            telemetry.addData("Loop ms", "%.1f", loopTime);
            telemetry.update();
        }
    }
}
