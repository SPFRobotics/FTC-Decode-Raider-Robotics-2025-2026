package org.firstinspires.ftc.teamcode.Game.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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
    
    // AprilTag centering variables
    private static final double CENTERING_THRESHOLD = 1.0; // degrees - stop centering when within this threshold
    private static final double MAX_CENTERING_SPEED = 0.5; // max power for centering
    private static final double CENTERING_GAIN = 0.02; // proportional gain for centering
    private boolean centeringEnabled = false;
    private int targetAprilTagId = -1; // -1 means any AprilTag

    // Constructor - initializes the limelight and IMU
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        
        imu = hardwareMap.get(IMU.class, "imu");
        // Fixed orientation for upside-down logo facing right
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN, 
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
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

    // Enable/disable auto-centering
    public void setCenteringEnabled(boolean enabled) {
        centeringEnabled = enabled;
    }

    // Check if centering is enabled
    public boolean isCenteringEnabled() {
        return centeringEnabled;
    }

    // Set target AprilTag ID (-1 for any tag)
    public void setTargetAprilTagId(int tagId) {
        targetAprilTagId = tagId;
    }

    // Get target AprilTag ID
    public int getTargetAprilTagId() {
        return targetAprilTagId;
    }

    /**
     * Calculate rotation speed needed to center on target
     * @return rotation speed (-1.0 to 1.0), or 0.0 if no target or centering disabled
     */
    public double getCenteringRotationSpeed() {
        if (!centeringEnabled) {
            return 0.0;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0.0; // No target detected
        }

        // Check if we're looking for a specific tag
        if (targetAprilTagId != -1) {
            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
                return 0.0; // No tags detected
            }
            // Check if the target tag is in the results
            boolean foundTargetTag = false;
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetAprilTagId) {
                    foundTargetTag = true;
                    break;
                }
            }
            if (!foundTargetTag) {
                return 0.0; // Target tag not found
            }
        }

        double tx = result.getTx();
        
        // If we're within the threshold, stop rotating
        if (Math.abs(tx) < CENTERING_THRESHOLD) {
            return 0.0;
        }

        // Calculate rotation speed using proportional control
        double rotationSpeed = tx * CENTERING_GAIN;
        
        // Clamp to max speed
        if (rotationSpeed > MAX_CENTERING_SPEED) {
            rotationSpeed = MAX_CENTERING_SPEED;
        } else if (rotationSpeed < -MAX_CENTERING_SPEED) {
            rotationSpeed = -MAX_CENTERING_SPEED;
        }

        return rotationSpeed;
    }

    /**
     * Check if robot is centered on target (within threshold)
     * @return true if centered, false otherwise
     */
    public boolean isCentered() {
        if (!centeringEnabled) {
            return false;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return false;
        }

        double tx = result.getTx();
        return Math.abs(tx) < CENTERING_THRESHOLD;
    }

    /**
     * Get the rotation angle needed to center on target (in degrees)
     * Positive values = rotate right (clockwise), Negative values = rotate left (counter-clockwise)
     * @return angle in degrees to rotate, or 0.0 if no target or already centered
     */
    public double getCenteringRotationAngle() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0.0; // No target detected
        }

        // Check if we're looking for a specific tag
        if (targetAprilTagId != -1) {
            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
                return 0.0; // No tags detected
            }
            // Check if the target tag is in the results
            boolean foundTargetTag = false;
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetAprilTagId) {
                    foundTargetTag = true;
                    break;
                }
            }
            if (!foundTargetTag) {
                return 0.0; // Target tag not found
            }
        }

        double tx = result.getTx();
        
        // If we're within the threshold, no rotation needed
        if (Math.abs(tx) < CENTERING_THRESHOLD) {
            return 0.0;
        }

        return tx; // Return the angle offset (positive = right, negative = left)
    }

    /**
     * Get the rotation angle needed to center on target (bypasses threshold check)
     * Use this for encoder-based centering to always get the actual offset
     * @return angle in degrees to rotate, or 0.0 if no target
     */
    public double getCenteringRotationAngleRaw() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0.0; // No target detected
        }

        // Check if we're looking for a specific tag
        if (targetAprilTagId != -1) {
            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
                return 0.0; // No tags detected
            }
            // Check if the target tag is in the results
            boolean foundTargetTag = false;
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetAprilTagId) {
                    foundTargetTag = true;
                    break;
                }
            }
            if (!foundTargetTag) {
                return 0.0; // Target tag not found
            }
        }

        // Return the raw tx value without threshold check
        return result.getTx();
    }

    /**
     * Check if a valid target is detected (for use with encoder-based centering)
     * @return true if target is detected and valid
     */
    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return false;
        }

        // Check if we're looking for a specific tag
        if (targetAprilTagId != -1) {
            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
                return false; // No tags detected
            }
            // Check if the target tag is in the results
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetAprilTagId) {
                    return true;
                }
            }
            return false; // Target tag not found
        }

        return true; // Any target is valid
    }

    /**
     * Get the estimated distance to the target in inches
     * Uses the botpose_MT2 Z value (forward distance) if available, otherwise estimates from ty
     * @return distance in inches, or -1 if no valid target
     */
    public double getDistanceToTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1.0;
        }

        // Try to get distance from botpose_MT2 (most accurate)
        try {
            Pose3D botPose = result.getBotpose_MT2();
            if (botPose != null) {
                double z = botPose.getPosition().z; // Z is forward distance in inches
                if (z > 0) {
                    return z;
                }
            }
        } catch (Exception e) {
            // Fall back to ty-based estimation
        }

        // Fallback: Estimate distance from ty (vertical angle)
        // This is less accurate but can work if botpose isn't available
        double ty = result.getTy();
        if (Math.abs(ty) > 0.1) {
            // Rough estimation: distance ≈ (target height - camera height) / tan(ty)
            // This requires knowing your camera and target heights
            // For now, return a default estimate based on ty
            // You may need to calibrate this for your specific setup
            double estimatedDistance = 24.0; // Default estimate in inches
            // You can uncomment and adjust this based on your camera height:
            // double cameraHeight = 10.0; // inches from ground
            // double targetHeight = 34.0; // inches (typical AprilTag height)
            // estimatedDistance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(ty));
            return estimatedDistance;
        }

        return -1.0; // No valid distance data
    }




}
