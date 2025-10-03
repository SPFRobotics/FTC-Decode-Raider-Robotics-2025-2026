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

        // Diagnostic info - always show
        telemetry.addLine("=== LIMELIGHT 3A DIAGNOSTICS ===");
        telemetry.addData("Result Null?", llResult == null ? "YES - CHECK CONNECTION" : "No");
        if (llResult != null) {
            telemetry.addData("Result Valid?", llResult.isValid() ? "YES" : "NO");
            telemetry.addData("Staleness", "%d ms", llResult.getStaleness());
        }

        if (llResult != null && llResult.isValid()) {
            // Get MegaTag2 pose (field positioning)
            Pose3D botPose = llResult.getBotpose_MT2();

            // Target Detection Data
            telemetry.addLine("=== LIMELIGHT 3A DATA ===");
            telemetry.addData("Target Detected", llResult.isValid() ? "YES" : "NO");
            
            // Targeting Information
            telemetry.addData("Tx (Horizontal Offset)", "%.2f°", llResult.getTx());
            telemetry.addData("Ty (Vertical Offset)", "%.2f°", llResult.getTy());
            telemetry.addData("Ta (Target Area)", "%.2f%%", llResult.getTa());
            
            // Robot Pose (MegaTag2 - Field Position)
            telemetry.addLine("\n--- Robot Position (MT2) ---");
            telemetry.addData("X Position", "%.2f inches", botPose.getPosition().x);
            telemetry.addData("Y Position", "%.2f inches", botPose.getPosition().y);
            telemetry.addData("Z Position", "%.2f inches", botPose.getPosition().z);
            telemetry.addData("Yaw", "%.2f°", botPose.getOrientation().getYaw());
            telemetry.addData("Pitch", "%.2f°", botPose.getOrientation().getPitch());
            telemetry.addData("Roll", "%.2f°", botPose.getOrientation().getRoll());
            
            // Color Detection (if using neural detector)
            if (llResult.getColorResults() != null && !llResult.getColorResults().isEmpty()) {
                telemetry.addLine("\n--- Color Detection ---");
                telemetry.addData("Colors Found", llResult.getColorResults().size());
            }
            
            // Detector Results (if using neural/fiducial detector)
            if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
                telemetry.addLine("\n--- AprilTag Detection ---");
                telemetry.addData("Tags Found", llResult.getFiducialResults().size());
                telemetry.addData("First Tag ID", llResult.getFiducialResults().get(0).getFiducialId());
            }
            
            // Capture Latency
            telemetry.addData("\nCapture Latency", "%d ms", llResult.getCaptureLatency());
            telemetry.addData("Target Latency", "%d ms", llResult.getTargetingLatency());
            telemetry.addData("Parse Latency", "%d ms", llResult.getParseLatency());
            
        } else {
            telemetry.addLine("\n=== NO VALID TARGETS ===");
            telemetry.addData("Status", "Searching for targets...");
            
            // Show what we're looking for
            if (llResult != null) {
                telemetry.addLine("\nTroubleshooting Tips:");
                telemetry.addData("1", "Check Limelight LEDs are ON");
                telemetry.addData("2", "Point at AprilTag (Pipeline 1)");
                telemetry.addData("3", "Ensure tag is well-lit");
                telemetry.addData("4", "Distance: 1-15 feet works best");
                telemetry.addData("5", "Check pipeline config at limelight.local:5801");
            }
        }
        
        // IMU Data
        telemetry.addLine("\n--- IMU Data ---");
        telemetry.addData("Robot Yaw", "%.2f°", headingOrientation.getYaw());
        telemetry.addData("Robot Pitch", "%.2f°", headingOrientation.getPitch());
        telemetry.addData("Robot Roll", "%.2f°", headingOrientation.getRoll());
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
