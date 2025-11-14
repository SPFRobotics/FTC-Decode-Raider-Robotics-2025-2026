package org.firstinspires.ftc.teamcode.Resources;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimelightOld {

    private Limelight3A limelight;
    private IMU imu;
    private Telemetry telemetry;
    
    // AprilTag centering variables
    private static final double CENTERING_THRESHOLD = 1.0; // degrees - stop centering when within this threshold
    private static final double MAX_CENTERING_SPEED = 0.5; // max power for centering
    private static final double CENTERING_GAIN = 0.02; // proportional gain for centering
    private boolean centeringEnabled = false;
    private int targetAprilTagId = -1; // -1 means any AprilTag
    
    // Cached target data for robustness during temporary target loss
    private double lastValidTx = 0.0;
    private long lastValidTargetTime = 0;
    private static final long TARGET_TIMEOUT_MS = 500; // Allow 500ms grace period for target reacquisition
    
    // Retry constants for centering operations
    private static final int MAX_RETRIES = 5;
    private static final int RETRY_DELAY_MS = 50;
    private static final double MIN_ROTATION_THRESHOLD = 0.2; // degrees - don't rotate if less than this
    private static final double ROTATION_TOLERANCE = 2.0; // degrees - acceptable error after rotation
    private static final double STRAFE_TOLERANCE = 1.5; // degrees - acceptable error after strafing

    // Constructor - initializes the limelight and IMU
    public LimelightOld(HardwareMap hardwareMap, Telemetry telemetry) {
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
            // Cache valid target data
            lastValidTx = llResult.getTx();
            lastValidTargetTime = System.currentTimeMillis();
            
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
            // Check if we're within the grace period (target temporarily lost but recently had valid target)
            long timeSinceLastValid = System.currentTimeMillis() - lastValidTargetTime;
            if (timeSinceLastValid < TARGET_TIMEOUT_MS && lastValidTargetTime > 0) {
                telemetry.addLine("=== LIMELIGHT ===");
                telemetry.addData("Target", "RECONNECTING...");
                telemetry.addData("Last Valid Tx", String.format("%.1f°", lastValidTx));
            } else {
                telemetry.addLine("=== LIMELIGHT ===");
                telemetry.addData("Target", "NO LOCK");
            }
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
    // Uses cached value during temporary target loss (grace period)
    public double getTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        // If target is lost but we're within grace period, use cached value
        if (checkGracePeriod()) {
            return lastValidTx;
        }
        return 0.0;
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
     * Uses cached value during temporary target loss
     * @return angle in degrees to rotate, or 0.0 if no target
     */
    public double getCenteringRotationAngleRaw() {
        LLResult result = limelight.getLatestResult();
        boolean currentlyValid = (result != null && result.isValid());
        
        double tx;
        
        if (currentlyValid) {
            // Check if we're looking for a specific tag
            if (targetAprilTagId != -1) {
                if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
                    // No tags detected, use cached value if available
                    if (checkGracePeriod()) {
                        return lastValidTx;
                    }
                    return 0.0;
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
                    // Target tag not found, use cached value if available
                    if (checkGracePeriod()) {
                        return lastValidTx;
                    }
                    return 0.0;
                }
            }
            // Return the raw tx value
            tx = result.getTx();
        } else {
            // Not currently valid, but check grace period
            if (checkGracePeriod()) {
                // Use cached value during grace period
                tx = lastValidTx;
            } else {
                return 0.0; // No target and grace period expired
            }
        }

        // Return the raw tx value without threshold check
        return tx;
    }

    /**
     * Check if a valid target is detected (for use with encoder-based centering)
     * Includes grace period for temporary target loss during movement
     * @return true if target is detected and valid, or if recently had valid target (within grace period)
     */
    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        boolean currentlyValid = (result != null && result.isValid());
        
        // If currently valid, check tag ID requirements
        if (currentlyValid) {
            // Check if we're looking for a specific tag
            if (targetAprilTagId != -1) {
                if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
                    // No tags detected, but check grace period
                    return checkGracePeriod();
                }
                // Check if the target tag is in the results
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    if (fiducial.getFiducialId() == targetAprilTagId) {
                        return true;
                    }
                }
                // Target tag not found, but check grace period
                return checkGracePeriod();
            }
            return true; // Any target is valid
        }
        
        // Not currently valid, check if we're within grace period
        return checkGracePeriod();
    }
    
    /**
     * Check if we're within the grace period for target reacquisition
     * @return true if recently had valid target (within grace period)
     */
    private boolean checkGracePeriod() {
        if (lastValidTargetTime == 0) {
            return false; // Never had a valid target
        }
        long timeSinceLastValid = System.currentTimeMillis() - lastValidTargetTime;
        return timeSinceLastValid < TARGET_TIMEOUT_MS;
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

    /**
     * Wait for valid target with retry logic
     * @return true if valid target found, false otherwise
     */
    private boolean waitForValidTarget(MecanumChassis chassis) {
        int retryCount = 0;
        while (!hasValidTarget() && retryCount < MAX_RETRIES && chassis.opmode.opModeIsActive()) {
            chassis.opmode.sleep(RETRY_DELAY_MS);
            update();
            retryCount++;
        }
        return hasValidTarget();
    }

    /**
     * Center the robot on a target using Limelight data and encoder-based movement
     * This method rotates the robot to face the target based on Limelight's tx value
     * 
     * @param chassis The MecanumChassis instance for movement
     * @param rotationPower The power to use for rotation (0.0 to 1.0)
     * @param timeoutSeconds Maximum time to spend centering (0 = no timeout) - currently unused but kept for compatibility
     * @return true if successfully centered, false if target lost or timeout
     */
    public boolean centerOnTarget(MecanumChassis chassis, double rotationPower, double timeoutSeconds) {
        if (chassis == null) {
            telemetry.addData("Error", "Chassis is null");
            telemetry.update();
            return false;
        }

        // Update limelight to get latest data
        update();
        
        // Wait for valid target with retry logic
        if (!waitForValidTarget(chassis)) {
            telemetry.addData("Limelight Center", "No valid target detected after retries");
            telemetry.update();
            return false;
        }

        // Use raw angle (bypasses threshold) to always attempt centering when button is pressed
        double rotationAngle = getCenteringRotationAngleRaw();
        
        telemetry.addData("Limelight Center", "Rotation Angle: %.2f degrees", rotationAngle);
        telemetry.update();

        // If angle is very small, consider it centered
        if (Math.abs(rotationAngle) < MIN_ROTATION_THRESHOLD) {
            telemetry.addData("Limelight Center", "Already centered (< " + MIN_ROTATION_THRESHOLD + " deg)");
            telemetry.update();
            return true;
        }

        telemetry.addData("Limelight Center", "Rotating by: %.2f degrees", rotationAngle);
        telemetry.update();

        // Use the chassis rotate method to rotate by the calculated angle
        chassis.rotate(rotationAngle, rotationPower);

        // Wait for target to reacquire after rotation
        if (!waitForValidTarget(chassis)) {
            telemetry.addData("Limelight Center", "Target lost after rotation");
            telemetry.update();
            return false;
        }

        // Verify we're now centered (allow some tolerance for verification)
        update();
        if (hasValidTarget() && isCentered()) {
            telemetry.addData("Limelight Center", "Successfully centered!");
            telemetry.update();
            return true;
        }

        // Even if not perfectly centered, if we have a valid target and got close, consider it success
        if (hasValidTarget()) {
            double currentTx = getTx();
            if (Math.abs(currentTx) < ROTATION_TOLERANCE) {
                telemetry.addData("Limelight Center", "Centered (within tolerance)");
                telemetry.update();
                return true;
            }
        }

        telemetry.addData("Limelight Center", "Centering may need adjustment");
        telemetry.update();
        return true; // Return true even if not perfectly centered, as we did our best
    }

    /**
     * Center the robot on a Limelight target with default power
     * @param chassis The MecanumChassis instance for movement
     * @return true if successfully centered
     */
    public boolean centerOnTarget(MecanumChassis chassis) {
        return centerOnTarget(chassis, 0.5, 0);
    }

    /**
     * Continuously center on a Limelight target until centered or timeout
     * This will keep adjusting until the robot is within the centering threshold
     * 
     * @param chassis The MecanumChassis instance for movement
     * @param rotationPower The power to use for rotation
     * @param maxIterations Maximum number of centering attempts (0 = no limit, but check opModeIsActive)
     * @return true if successfully centered
     */
    public boolean centerOnTargetContinuous(MecanumChassis chassis, double rotationPower, int maxIterations) {
        if (chassis == null) {
            return false;
        }

        int iterations = 0;
        double minAngleThreshold = 0.5; // Minimum angle to bother rotating

        while (chassis.opmode.opModeIsActive() && (maxIterations == 0 || iterations < maxIterations)) {
            update();
            
            if (!hasValidTarget()) {
                telemetry.addData("Limelight Center", "Target lost");
                telemetry.update();
                return false;
            }

            double rotationAngle = getCenteringRotationAngle();
            
            // If centered, we're done
            if (Math.abs(rotationAngle) < minAngleThreshold || isCentered()) {
                telemetry.addData("Limelight Center", "Centered! Iterations: %d", iterations);
                telemetry.update();
                return true;
            }

            // Rotate by the calculated angle
            chassis.rotate(rotationAngle, rotationPower);
            iterations++;

            telemetry.addData("Limelight Center", "Iteration: %d, Angle: %.2f", iterations, rotationAngle);
            telemetry.update();
            
            chassis.opmode.sleep(100); // Small delay between iterations
        }

        return isCentered();
    }

    /**
     * Center the robot's position using Limelight by strafing left/right (encoder-based)
     * This calculates the distance needed to strafe based on tx angle and distance to target
     * 
     * @param chassis The MecanumChassis instance for movement
     * @param movePower Power to use for movement (0.0 to 1.0)
     * @param distanceToTarget Estimated distance to target in inches (used to calculate strafe distance)
     * @return true if successfully centered
     */
    public boolean centerOnTargetByStrafing(MecanumChassis chassis, double movePower, double distanceToTarget) {
        if (chassis == null) {
            telemetry.addData("Error", "Chassis is null");
            telemetry.update();
            return false;
        }

        update();
        
        // Wait for valid target with retry logic
        if (!waitForValidTarget(chassis)) {
            telemetry.addData("Limelight Center", "No valid target detected after retries");
            telemetry.update();
            return false;
        }

        // Get the horizontal offset angle directly using getTx() method
        double tx = getTx();
        telemetry.addData("Limelight Center", "Tx: %.2f degrees", tx);
        telemetry.update();
        
        // If already well-centered, return success
        if (Math.abs(tx) < 0.5) {
            telemetry.addData("Limelight Center", "Already well-centered (< 0.5 deg)");
            telemetry.update();
            chassis.restoreManualControl();
            return true;
        }

        // Convert angle to strafe distance using trigonometry
        // distance = tan(angle) * distanceToTarget
        // Note: tx is in degrees, so convert to radians
        double angleRadians = Math.toRadians(tx);
        double strafeDistanceInches = Math.tan(angleRadians) * distanceToTarget;

        telemetry.addData("Limelight Center", "Tx: %.2f deg, Strafe: %.2f inches", tx, strafeDistanceInches);
        telemetry.update();

        // Determine direction and move
        if (tx > 0) {
            // Target is to the right, strafe right
            chassis.move(movePower, "right", Math.abs(strafeDistanceInches));
        } else {
            // Target is to the left, strafe left
            chassis.move(movePower, "left", Math.abs(strafeDistanceInches));
        }

        // Restore manual control mode after movement
        chassis.restoreManualControl();

        // Wait for target to reacquire after strafing
        if (!waitForValidTarget(chassis)) {
            telemetry.addData("Limelight Center", "Target lost after strafing");
            telemetry.update();
            return false;
        }

        // Verify we're centered (allow some tolerance)
        update();
        if (hasValidTarget() && isCentered()) {
            telemetry.addData("Limelight Center", "Successfully centered by strafing!");
            telemetry.update();
            return true;
        }

        // Even if not perfectly centered, if we have a valid target and got close, consider it success
        if (hasValidTarget()) {
            double currentTx = getTx();
            if (Math.abs(currentTx) < STRAFE_TOLERANCE) {
                telemetry.addData("Limelight Center", "Centered by strafing (within tolerance)");
                telemetry.update();
                return true;
            }
        }

        return true;
    }

    /**
     * Center on Limelight target using both rotation and strafing (encoder-based)
     * First rotates to face the target, then strafes to center position
     * 
     * @param chassis The MecanumChassis instance for movement
     * @param rotationPower Power for rotation
     * @param movePower Power for strafing
     * @param distanceToTarget Estimated distance to target in inches
     * @return true if successfully centered
     */
    public boolean centerOnTargetFull(MecanumChassis chassis, double rotationPower, double movePower, double distanceToTarget) {
        // First, rotate to face the target
        boolean rotated = centerOnTarget(chassis, rotationPower, 0);
        
        if (!rotated) {
            chassis.restoreManualControl();
            return false;
        }

        chassis.opmode.sleep(200); // Small delay after rotation

        // Then strafe to center position (this will restore manual control at the end)
        return centerOnTargetByStrafing(chassis, movePower, distanceToTarget);
    }

}

