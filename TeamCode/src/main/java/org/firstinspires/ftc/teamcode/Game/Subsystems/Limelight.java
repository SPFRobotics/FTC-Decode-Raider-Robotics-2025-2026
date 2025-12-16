package org.firstinspires.ftc.teamcode.Game.Subsystems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

public class Limelight {
    public Limelight3A limelight;

    // Motif IDs
    private static final int[] VALID_MOTIF_IDS = {21, 22, 23};

    // Shooting April Tags
    private static final int[] SHOOTING_TAG_IDS = {20, 24};

    //Limelight Constructor
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    //Starts Limelight

    
    public void start() {
        limelight.start();
    }

    //Stops Limelight
    public void stop() {
        limelight.stop();
    }

    //Gets Motif ID, if no motif is detected, returns -1
    public int getMotifId() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return -1;
        }

        // Check each detected fiducial to see if it matches a valid motif ID
        for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
            int fiducialId = fiducial.getFiducialId();

            // Check if this fiducial ID is one of the valid motif IDs
            for (int motifId : VALID_MOTIF_IDS) {
                if (fiducialId == motifId) {
                    return motifId;
                }
            }
        }

        // No valid motif detected
        return -1;
    }

    //Gets Robot Field Coordinates
    public Pose3D botpose() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        // Get the robot's pose from the Limelight
        Pose3D botpose = result.getBotpose();

        return botpose;
    }

    //Gets Shooting April Tag ID
    public int getShootingAprilTagId() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return -1;
        }

        // Check for shooting tag id
        for (LLResultTypes.FiducialResult fiducial : fiducialResults) {
            int fiducialId = fiducial.getFiducialId();


            for (int tagId : SHOOTING_TAG_IDS) {
                if (fiducialId == tagId) {
                    return tagId;
                }
            }
        }

        return -1;
    }

    //Gets latest result
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }


    //Checks for valid result
    public boolean hasValidResult() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    // Swithc Limelight Pipeline
    public void setPipeline(int pipeline) {

        if (pipeline > 0 && pipeline < 9) {
            limelight.pipelineSwitch(pipeline);
        }
    }
}

