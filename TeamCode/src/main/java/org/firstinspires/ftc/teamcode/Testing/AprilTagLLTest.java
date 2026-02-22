package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class AprilTagLLTest {

    public enum Motif {
        GPP,   // id 21
        PGP,   // id 22
        PPG,   // id 23
        UNKNOWN
    }

    private  Limelight3A limelight;

    public AprilTagLLTest (Limelight3A limelight) {
        this.limelight = limelight;
    }

    /**
     * Reads Limelight and returns the current motif.
     * Chooses whichever of tags 21/22/23 has the largest area.
     */
    public Motif getMotif() {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Motif.UNKNOWN;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return Motif.UNKNOWN;
        }

        int bestId = -1;
        double bestArea = -1;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            int id = fr.getFiducialId();

            if (id == 21 || id == 22 || id == 23) {
                double area = fr.getTargetArea();

                if (area > bestArea) {
                    bestArea = area;
                    bestId = id;
                }
            }
        }

        return idToMotif(bestId);
    }

    private Motif idToMotif(int id) {
        switch (id) {
            case 21: return Motif.GPP;
            case 22: return Motif.PGP;
            case 23: return Motif.PPG;
            default: return Motif.UNKNOWN;
        }
    }
}