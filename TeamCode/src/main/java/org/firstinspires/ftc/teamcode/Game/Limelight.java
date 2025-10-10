package org.firstinspires.ftc.teamcode.Game;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Limelight {

    private Limelight3A limelight;
    private IMU imu;
    private Telemetry telemetry;

    // Constructor - initializes the limelight and IMU
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP, 
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public void start() {
        limelight.start();
    }

    public void update() {
        // Update robot orientation from IMU
        YawPitchRollAngles headingOrientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(headingOrientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // Get MegaTag2 pose (field positioning)
            Pose3D botPose = llResult.getBotpose_MT2();

            // Target Detection
            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Target", "LOCKED");
            telemetry.addData("Tx", String.format("%.1f°", llResult.getTx()));
            telemetry.addData("Ty", String.format("%.1f°", llResult.getTy()));
            
            // Robot Position
            telemetry.addData("X", String.format("%.1f", botPose.getPosition().x));
            telemetry.addData("Y", String.format("%.1f", botPose.getPosition().y));
            telemetry.addData("Heading", String.format("%.1f°", botPose.getOrientation().getYaw()));
            
            // AprilTag Info
            if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
                telemetry.addData("Tag ID", llResult.getFiducialResults().get(0).getFiducialId());
            }
            
        } else {
            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Target", "NO LOCK");
        }
    }

    // Getter for latest result (if needed elsewhere)
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    // Check if valid target is detected
    public boolean hasTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    // Get horizontal offset (useful for alignment)
    public double getTx() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTx() : 0.0;
    }

    // Get vertical offset
    public double getTy() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }

    // Get target area (useful for distance estimation)
    public double getTa() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTa() : 0.0;
    }

    // Switch pipeline
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    // Get robot pose from MegaTag2
    public Pose3D getRobotPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        return null;
    }

    // Stop limelight
    public void stop() {
        limelight.stop();
    }

    // Get number of AprilTags detected
    public int getAprilTagCount() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            return result.getFiducialResults().size();
        }
        return 0;
    }

    // Get the ID of the first detected AprilTag (-1 if none)
    public int getFirstAprilTagId() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && 
            result.getFiducialResults() != null && 
            !result.getFiducialResults().isEmpty()) {
            return result.getFiducialResults().get(0).getFiducialId();
        }
        return -1;
    }

    // Check if Limelight is connected and responding
    public boolean isConnected() {
        return limelight.getLatestResult() != null;
    }

    // Get data staleness (how old the data is)
    public double getStaleness() {
        LLResult result = limelight.getLatestResult();
        return (result != null) ? result.getStaleness() : -1.0;
    }
}
