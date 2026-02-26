package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "AprilTagLLTest (Motif)")
public class AprilTagLLTest extends LinearOpMode {

    private Limelight3A limelight;

    // CHANGE THIS if your AprilTag/Fiducial pipeline is not #0
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    // Motif mapping
    private enum Motif { GPP, PGP, PPG, UNKNOWN }

    private Motif idToMotif(int id) {
        switch (id) {
            case 21: return Motif.GPP;
            case 22: return Motif.PGP;
            case 23: return Motif.PPG;
            default: return Motif.UNKNOWN;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(50);

        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        telemetry.addLine("AprilTag Motif Test Ready.");
        telemetry.addLine("Point Limelight at OBELISK (IDs 21/22/23).");
        telemetry.addLine("If Motif stays UNKNOWN: pipeline must be Fiducial/AprilTag.");
        telemetry.update();

        // Keep last stable motif so it doesn't flicker to UNKNOWN
        Motif lockedMotif = Motif.UNKNOWN;
        int bestId = -1;
        double bestArea = 0;

        // INIT LOOP (before Start)
        while (!isStarted() && !isStopRequested()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Pipeline", "Index=%d Type=%s",
                    status.getPipelineIndex(), status.getPipelineType());
            telemetry.addData("LL", "Temp=%.1fC CPU=%.1f%% FPS=%d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());

            LLResult result = limelight.getLatestResult();

            if (result == null) {
                telemetry.addLine("Result: NULL (no frame yet)");
            } else if (!result.isValid()) {
                telemetry.addLine("Result: INVALID");
            } else {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                int count = (fiducials == null) ? 0 : fiducials.size();
                telemetry.addData("Fiducials Seen", count);

                // Print every tag we see (debug)
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        telemetry.addData("Tag",
                                "ID=%d area=%.2f x=%.2f y=%.2f",
                                fr.getFiducialId(),
                                fr.getTargetArea(),
                                fr.getTargetXDegrees(),
                                fr.getTargetYDegrees());
                    }

                    // Pick best motif tag (21/22/23) by largest area
                    int localBestId = -1;
                    double localBestArea = -1;

                    for (LLResultTypes.FiducialResult fr : fiducials) {
                        int id = fr.getFiducialId();
                        if (id == 21 || id == 22 || id == 23) {
                            double area = fr.getTargetArea();
                            if (area > localBestArea) {
                                localBestArea = area;
                                localBestId = id;
                            }
                        }
                    }

                    // Update stable motif if we found one
                    if (localBestId != -1) {
                        bestId = localBestId;
                        bestArea = localBestArea;
                        lockedMotif = idToMotif(bestId);
                    }
                }
            }

            telemetry.addData("Best Motif Tag ID", bestId);
            telemetry.addData("Best Motif Area", "%.2f", bestArea);
            telemetry.addData("MOTIF (LIVE)", lockedMotif);

            telemetry.addLine("---- Motif ID Map ----");
            telemetry.addLine("21 -> GPP | 22 -> PGP | 23 -> PPG");

            telemetry.update();
        }

        // Start pressed — lock it and keep displaying
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("LOCKED MOTIF", lockedMotif);
            telemetry.addData("LOCKED TAG ID", bestId);
            telemetry.addData("LOCKED AREA", "%.2f", bestArea);
            telemetry.update();
        }

        limelight.stop();
    }
}