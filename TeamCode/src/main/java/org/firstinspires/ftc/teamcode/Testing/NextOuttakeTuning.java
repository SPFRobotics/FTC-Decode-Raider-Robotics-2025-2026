package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.NextFTC.NextOuttake;

import java.util.List;

@TeleOp(name = "NextOuttake Tuning", group = "Testing")
public class NextOuttakeTuning extends LinearOpMode {

    private final double[] rpmPresets = {0, 1000, 1500, 2000, 2600, 3000, 3300};
    private int presetIndex = 0;
    private double targetRPM = 0;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevBack = false;

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        NextOuttake outtake = NextOuttake.INSTANCE;
        outtake.initialize();

        telemetry.addLine("=== NextOuttake PID Tuner ===");
        telemetry.addLine("Adjust kP/kI/kD/kV/kS in Dashboard under NextOuttake");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD UP/DOWN : Cycle RPM presets");
        telemetry.addLine("  A            : Set to farRPM");
        telemetry.addLine("  B            : Set to closeRPM");
        telemetry.addLine("  X            : Stop (0 RPM)");
        telemetry.addLine("  BACK         : Re-apply PID gains from Dashboard");
        telemetry.addLine("  RIGHT STICK Y: Manual RPM adjust (+/- 500)");
        telemetry.update();

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive()) {
            double loopTime = loopTimer.milliseconds();
            loopTimer.reset();

            if (gamepad1.back && !prevBack) {
                outtake.initialize();
                if (targetRPM > 0) {
                    outtake.setRPM(targetRPM);
                }
            }
            prevBack = gamepad1.back;

            if (gamepad1.dpad_up && !prevDpadUp) {
                presetIndex = Math.min(presetIndex + 1, rpmPresets.length - 1);
                targetRPM = rpmPresets[presetIndex];
                outtake.setRPM(targetRPM);
            }
            prevDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !prevDpadDown) {
                presetIndex = Math.max(presetIndex - 1, 0);
                targetRPM = rpmPresets[presetIndex];
                outtake.setRPM(targetRPM);
            }
            prevDpadDown = gamepad1.dpad_down;

            if (gamepad1.a && !prevA) {
                targetRPM = NextOuttake.farRPM;
                outtake.setRPM(targetRPM);
            }
            prevA = gamepad1.a;

            if (gamepad1.b && !prevB) {
                targetRPM = NextOuttake.closeRPM;
                outtake.setRPM(targetRPM);
            }
            prevB = gamepad1.b;

            if (gamepad1.x) {
                targetRPM = 0;
                outtake.stop();
            }

            if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                targetRPM = Math.max(0, targetRPM - gamepad1.right_stick_y * 10);
                outtake.setRPM(targetRPM);
            }

            outtake.periodic();

            double actualRPM = outtake.getRPM();
            double error = targetRPM - actualRPM;
            double errorPct = targetRPM != 0 ? (error / targetRPM) * 100 : 0;

            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Actual RPM", "%.0f", actualRPM);
            telemetry.addData("Error", "%.0f (%.1f%%)", error, errorPct);
            telemetry.addData("Motor Power", "%.3f", outtake.getPower());
            telemetry.addLine("");
            telemetry.addData("Preset", "[%d] = %.0f", presetIndex, rpmPresets[presetIndex]);
            telemetry.addLine("");
            telemetry.addData("kP", NextOuttake.kP);
            telemetry.addData("kI", NextOuttake.kI);
            telemetry.addData("kD", NextOuttake.kD);
            telemetry.addData("kV", NextOuttake.kV);
            telemetry.addData("kS", NextOuttake.kS);
            telemetry.addLine("");
            telemetry.addData("Loop ms", "%.1f", loopTime);
            telemetry.addData("Runtime", "%.1f s", runtime.seconds());
            telemetry.update();
        }

        outtake.stop();
        outtake.periodic();
    }
}
