package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.Button;
import org.firstinspires.ftc.teamcode.Resources.VERYheavilyInspiredSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.ColorFetch;
import org.firstinspires.ftc.teamcode.Subsystems.KickerSpindex;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@TeleOp(name = "Stolen Spindex Test", group = "Testing")
public class StolenSpindexTst extends LinearOpMode {

    private Button modeToggle     = new Button();
    private Button rightBumperBtn = new Button();
    private Button leftBumperBtn  = new Button();
    private Button autoLoadToggle = new Button();

    @Override
    public void runOpMode() {
        // Subsystems
        KickerSpindex kicker      = new KickerSpindex(hardwareMap);
        ColorFetch    colorSensor  = new ColorFetch(hardwareMap);
        Outtake       outtake      = new Outtake(hardwareMap, true);

        VERYheavilyInspiredSpindex spindex =
                new VERYheavilyInspiredSpindex(hardwareMap, kicker, colorSensor, outtake);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        autoLoadToggle.changeState(true);

        telemetry.addLine("Ready -- press START");
        telemetry.update();
        waitForStart();

        ElapsedTime loopTime = new ElapsedTime();

        while (opModeIsActive()) {
            loopTime.reset();

            // ===== Gamepad2: Spindex controls (same mapping as TeleOpMain) =====

            // Right bumper -> next chamber
            if (rightBumperBtn.press(gamepad2.right_bumper)) {
                if (!spindex.isOuttakeing()) {
                    autoLoadToggle.changeState(false);
                }
                spindex.addIndex();
            }

            // Left bumper -> previous chamber
            if (leftBumperBtn.press(gamepad2.left_bumper)) {
                if (!spindex.isOuttakeing()) {
                    autoLoadToggle.changeState(false);
                }
                spindex.subtractIndex();
            }

            // Circle -> toggle intake / outtake mode
            spindex.setMode(modeToggle.toggle(gamepad2.circle));

            // Cross -> kick ball (when outtaking and flywheel spinning)
            boolean crossWasPressed = gamepad2.crossWasPressed();
            kicker.automate(crossWasPressed && spindex.isOuttakeing());
            if (crossWasPressed && spindex.isOuttakeing() && outtake.getPower() != 0) {
                spindex.clearBall(spindex.getIndex());
            }

            // Triangle -> toggle auto-load
            spindex.setAutoLoadMode(
                    autoLoadToggle.toggle(gamepad2.triangle) && !spindex.isOuttakeing());
            spindex.autoLoad(colorSensor);

            // Move the spindex (P-controller tick)
            if (spindex.isOuttakeing()) {
                spindex.moveToPos(
                        VERYheavilyInspiredSpindex.SpindexConfig.outtakePos[spindex.getIndex()],
                        true);
            } else {
                spindex.moveToPos(
                        VERYheavilyInspiredSpindex.SpindexConfig.intakePos[spindex.getIndex()],
                        true);
            }

            // D-pad up/down -> outtake RPM
            if (gamepad2.dpad_up) {
                outtake.setRPM(Outtake.OuttakeConfig.farRPM);
            } else if (gamepad2.dpad_down) {
                outtake.setRPM(Outtake.OuttakeConfig.closeRPM);
            } else if (gamepad2.touchpad) {
                outtake.setRPM(0);
            }

            // ===== Telemetry =====
            spindex.postTelemetry(telemetry);
            telemetry.addData("Loop ms", "%.1f", loopTime.milliseconds());
            telemetry.addData("Outtake RPM", "%.0f", outtake.getRPM());
            telemetry.addData("Distance", "%.1f", colorSensor.getDistance());
            telemetry.addData("Colour Hue", "%.0f", colorSensor.getHue());
            telemetry.update();
        }
    }
}
