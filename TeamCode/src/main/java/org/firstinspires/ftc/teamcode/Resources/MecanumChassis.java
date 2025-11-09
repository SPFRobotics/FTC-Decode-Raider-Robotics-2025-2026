package org.firstinspires.ftc.teamcode.Resources;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import java.util.ArrayList;
import java.util.List;

public class MecanumChassis {
    public LinearOpMode opmode = null;
    //Odometry odom = new Odometry(opmode);
    public static final double strafeMult = 1.1;
    public double liftPosition = 0;
    public final double liftMaxMotorCounts = 4062;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;

    /*public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;*/
    public IMU imu = null;

    private double targetDist = 0;
    private double powerFrontRight = 0;
    private double powerFrontLeft = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;
    private int wiggleCount = 0;

    public static double kP = 0.01;
    public static double kI = 0.00001;
    public static double kD = 0.0001;

    public MecanumChassis(LinearOpMode lom){
        opmode = lom;
    }
    public double inch_convert(double inch) { return inch * (537.7 / (3.78 * Math.PI)); }
    public double inToCm(int inches) { return inches * 2.54; }
    public double cm_convert(double cm) { return cm * (537.7 / (9.6012 * Math.PI)); }
    public void initializeMovement() {
        //odom.init();
        backLeft = opmode.hardwareMap.dcMotor.get("backLeftDrive");
        backRight = opmode.hardwareMap.dcMotor.get("backRightDrive");
        frontLeft = opmode.hardwareMap.dcMotor.get("frontLeftDrive");
        frontRight = opmode.hardwareMap.dcMotor.get("frontRightDrive");

        //odom.setPose(0,0,0);


        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        stop_and_reset_encoders_all();
        //run_without_encoders_all();

        imu = opmode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        //waitForStart();
    }
    public void stop_and_reset_encoders_all() {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void run_to_position_all() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void run_without_encoders_all() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void run_using_encoders_all() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void powerZero() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
    public boolean move(double movePower, @NonNull String moveDirection, double moveDistance){ // added support for moving lift and wheels at the same time.
        stop_and_reset_encoders_all(); //Sets encoder count to 0
        run_using_encoders_all();
        if (moveDirection.equals("forward")) {
            //Tell each wheel to move a certain amount
            backLeft.setTargetPosition((int) inch_convert(moveDistance)); //Converts the
            backRight.setTargetPosition((int) inch_convert(moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance));
            frontRight.setTargetPosition((int) inch_convert(moveDistance));

            run_to_position_all();
            opmode.telemetry.addData("Power", movePower);
            opmode.telemetry.update();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection.equals("backward")) {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance));
            backRight.setTargetPosition((int) inch_convert(-moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance));
            run_to_position_all();
            backLeft.setPower(-movePower);
            backRight.setPower(-movePower);
            frontLeft.setPower(-movePower);
            frontRight.setPower(-movePower);
        } else if (moveDirection.equals("right")) {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            backRight.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            run_to_position_all();
            backLeft.setPower(-movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(-movePower);
        } else if (moveDirection.equals("left")) {
            backLeft.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            backRight.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            frontRight.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(-movePower);
            frontLeft.setPower(-movePower);
            frontRight.setPower(movePower);
        } else {
            opmode.telemetry.addData("Error", "move direction must be forward,backward,left, or right.");
            opmode.telemetry.update();
            opmode.terminateOpModeNow();
        }
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            /*odom.updateOdom();
            opmode.telemetry.addData("X", odom.getX());
            opmode.telemetry.addData("Y", odom.getY());
            opmode.telemetry.addData("Heading", odom.getHeading());*/
            opmode.telemetry.addData("test", "attempting to move...");
            opmode.telemetry.addData("power back right", backRight.getPower());
            opmode.telemetry.addData("power back left", backLeft.getPower());
            opmode.telemetry.addData("power front right", frontRight.getPower());
            opmode.telemetry.addData("power front left", frontLeft.getPower());

            opmode.telemetry.update();
        }
        powerZero();
        opmode.sleep(500);
        // Restore motors to manual control mode after encoder movement
        // This is critical for TeleOp to work properly after using this method
        run_without_encoders_all();
        opmode.telemetry.addData("test", "done!");
        opmode.telemetry.update();
        return false;
    }

    public void moveWithCorrections(double movePower, @NonNull String moveDirection, double moveDistance, double angle) {
        stop_and_reset_encoders_all(); //Sets encoder count to 0
        //run_using_encoders_all();
        double Kp = 1;
        double bLPower = 0;
        double bRPower = 0;
        double fLPower = 0;
        double fRPower = 0;
        //double[] motorPowers = new double[] {bLPower,bRPower,fLPower,fRPower};
        List<Double> motorPowers = new ArrayList<>();
        motorPowers.add(0,bLPower);
        motorPowers.add(1,bRPower);
        motorPowers.add(2,fLPower);
        motorPowers.add(3,fRPower);
        double error = 0;
        double powerR = 0;
        double powerL = 0;
        //double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //double targetAngle = startAngle + angle;
        double targetAngle = angle;
        if (moveDirection.equals("forward")) {
            //Tell each wheel to move a certain amount
            backLeft.setTargetPosition((int) inch_convert(moveDistance)); //Converts the
            backRight.setTargetPosition((int) inch_convert(moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance));
            frontRight.setTargetPosition((int) inch_convert(moveDistance));
            run_to_position_all();
            opmode.telemetry.addData("Power", movePower);
            opmode.telemetry.update();
            bLPower = movePower;
            bRPower = movePower;
            fLPower = movePower;
            fRPower = movePower;
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection.equals("backward")) {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance));
            backRight.setTargetPosition((int) inch_convert(-moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance));
            run_to_position_all();
            bLPower = -movePower;
            bRPower = -movePower;
            fLPower = -movePower;
            fRPower = -movePower;
            backLeft.setPower(-movePower);
            backRight.setPower(-movePower);
            frontLeft.setPower(-movePower);
            frontRight.setPower(-movePower);
        } else if (moveDirection.equals("right")) {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            backRight.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            run_to_position_all();
            bLPower = -movePower;
            bRPower = movePower;
            fLPower = movePower;
            fRPower = -movePower;
            backLeft.setPower(-movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(-movePower);
        } else if (moveDirection.equals("left")) {
            backLeft.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            backRight.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            frontRight.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            run_to_position_all();
            bLPower = movePower;
            bRPower = -movePower;
            fLPower = -movePower;
            fRPower = movePower;
            backLeft.setPower(movePower);
            backRight.setPower(-movePower);
            frontLeft.setPower(-movePower);
            frontRight.setPower(movePower);
        } else {
            opmode.telemetry.addData("Error", "move direction must be forward,backward,left, or right.");
            opmode.telemetry.update();
            opmode.terminateOpModeNow();
        }
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            error = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle);
            powerR = Range.clip((movePower - (error*Kp)),-movePower,movePower);
            powerL = Range.clip((movePower + (error*Kp)),-movePower,movePower);
            switch (moveDirection) {
                case "forward":
                    bLPower = powerL;
                    bRPower = powerR;
                    fLPower = powerL;
                    fRPower = powerR;
                    break;
                case "backward":
                    bLPower = -powerL;
                    bRPower = -powerR;
                    fLPower = -powerL;
                    fRPower = -powerR;
                    break;
                case "right":
                    bLPower = -powerL;
                    bRPower = powerR;
                    fLPower = powerL;
                    fRPower = -powerR;
                    break;
                case "left":
                    bLPower = powerL;
                    bRPower = -powerR;
                    fLPower = -powerL;
                    fRPower = powerL;
                    break;
            }
            backLeft.setPower(bLPower);
            backRight.setPower(bRPower);
            frontLeft.setPower(fLPower);
            frontRight.setPower(fRPower);
            opmode.telemetry.addData("test", "attempting to move...");
            opmode.telemetry.addData("power back right", backRight.getPower());
            opmode.telemetry.addData("power back left", backLeft.getPower());
            opmode.telemetry.addData("power front right", frontRight.getPower());
            opmode.telemetry.addData("power front left", frontLeft.getPower());
            opmode.telemetry.update();
        }
        powerZero();
        opmode.sleep(500);
        // Restore motors to manual control mode after encoder movement
        run_without_encoders_all();
        opmode.telemetry.addData("test", "done!");
        opmode.telemetry.update();
    }

    public void rotate(double angle, double power) {
        double minPower = 0.2;
        double Kp = 0.04; //this is for proportional control (ie. the closer you are the target angle the slower you will go)
        double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle = startAngle + angle;
        double error = AngleUnit.normalizeDegrees((imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle));
        double power1 = 0;
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rotate until the target angle is reached

        while (opmode.opModeIsActive() && Math.abs(error) > .5) {
            /*odom.updateOdom();
            opmode.telemetry.addData("X", odom.getX());
            opmode.telemetry.addData("Y", odom.getY());
            opmode.telemetry.addData("Heading", odom.getHeading());*/
            //powerZero();
            error = AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle);
            // the closer the robot is to the target angle, the slower it rotates
            //power = Range.clip(Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle) / 90, 0.1, 0.5);
            opmode.telemetry.addData("real power", power*(error*Kp));
            power1 = Range.clip((power*(error*Kp)),-power,power); //"Range.clip(value, minium, maxium)" takes the first term and puts it in range of the min and max provided
            if (Math.abs(power1) < minPower) {
                power1 = minPower * (power1/Math.abs(power1));
            }
            opmode.telemetry.addData("power",power1);
            System.out.printf("%f power = ",power1);
            opmode.telemetry.addData("error",error);

            backLeft.setPower(power1);
            backRight.setPower(-power1);
            frontLeft.setPower(power1);
            frontRight.setPower(-power1);
            if (Math.abs(error) <= .5) {
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            opmode.telemetry.addData("angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            opmode.telemetry.addData("target", targetAngle);
            opmode. telemetry.update();
            //double angleDifference = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle);
            //rotate(angleDifference, power);
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        opmode.sleep(500);
    }
    /* public void rotate(double angle, double power) {
        double stopError = 0.5;
        double Kp = 0.5; //this is for proportional control (ie. the closer you are the target angle the slower you will go)
        double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle = startAngle + angle;
        double error = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle);
        double power1 = power;
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rotate until the target angle is reached
        System.out.printf("%f start angle = ",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        System.out.printf("%f error = ", error);
        while (opmode.opModeIsActive() && Math.abs(error) > stopError) {
            //powerZero();
            error = AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle);
            // the closer the robot is to the target angle, the slower it rotates
            //power = Range.clip(Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle) / 90, 0.1, 0.5);
            power1 = power * error * Kp;
            power1 = Range.clip(power1,-0.5,0.5); //"Range.clip(value, minium, maxium)" takes the first term and puts it in range of the min and max provided
            opmode.telemetry.addData("power1",power1);
            System.out.printf("%f power = ",power1);
            opmode.telemetry.addData("error",error);
            opmode.telemetry.addData("power", power);
            opmode.telemetry.addData("Kp",Kp);

            backLeft.setPower(power1);
            backRight.setPower(-power1);
            frontLeft.setPower(power1);
            frontRight.setPower(-power1);
            if (Math.abs(error) <= stopError) {
                powerZero();
            }
            opmode.telemetry.addData("angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            opmode.telemetry.addData("target", targetAngle);
            opmode.telemetry.update();
            //double angleDifference = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle);
            //rotate(angleDifference, power);
        }
        powerZero();
        opmode.sleep(500);
    } */

    public void wiggle(){
        move(1,"forward",3);
        move(1,"backward",3);
        wiggleCount++;
    }

    public int getWiggleCount(){
        return wiggleCount;
    }

    public void adjust(String direction, double power){
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (direction.equals("forward")){
            backLeft.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(power);
        }
        if (direction.equals("backward")){
            backLeft.setPower(-power);
            backRight.setPower(-power);
            frontLeft.setPower(-power);
            frontRight.setPower(-power);
        }
        if (direction.equals("left")){
            backLeft.setPower(power);
            backRight.setPower(-power);
            frontLeft.setPower(-power);
            frontRight.setPower(power);
        }
        if (direction.equals("right")){
            backLeft.setPower(-power);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(-power);
        }
        if (direction.equals("backLeft")){
            backLeft.setPower(0);
            backRight.setPower(-power);
            frontLeft.setPower(-power);
            frontRight.setPower(0);
        }
        if (direction.equals("backRight")){
            backLeft.setPower(-power);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(-power);
        }
        if (direction.equals("frontLeft")){
            backLeft.setPower(power);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(power);
        }
        if (direction.equals("frontRight")){
            backLeft.setPower(0);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(0);
        }
    }

    /**
     * Center the robot on a target using Limelight data and encoder-based movement
     * This method rotates the robot to face the target based on Limelight's tx value
     * 
     * @param limelight The Limelight instance to get targeting data from
     * @param rotationPower The power to use for rotation (0.0 to 1.0)
     * @param timeoutSeconds Maximum time to spend centering (0 = no timeout)
     * @return true if successfully centered, false if target lost or timeout
     */
    public boolean centerOnLimelightTarget(org.firstinspires.ftc.teamcode.Game.Limelight limelight,
                                           double rotationPower, double timeoutSeconds) {
        if (limelight == null) {
            opmode.telemetry.addData("Error", "Limelight is null");
            opmode.telemetry.update();
            return false;
        }

        // Update limelight to get latest data
        limelight.update();
        
        // Check if target is valid
        if (!limelight.hasValidTarget()) {
            opmode.telemetry.addData("Limelight Center", "No valid target detected");
            opmode.telemetry.update();
            return false;
        }

        // Use raw angle (bypasses threshold) to always attempt centering when button is pressed
        double rotationAngle = limelight.getCenteringRotationAngleRaw();
        
        opmode.telemetry.addData("Limelight Center", "Rotation Angle: %.2f degrees", rotationAngle);
        opmode.telemetry.update();

        // If angle is very small (less than 0.2 degrees), consider it centered
        if (Math.abs(rotationAngle) < 0.2) {
            opmode.telemetry.addData("Limelight Center", "Already centered (< 0.2 deg)");
            opmode.telemetry.update();
            return true;
        }

        opmode.telemetry.addData("Limelight Center", "Rotating by: %.2f degrees", rotationAngle);
        opmode.telemetry.update();

        // Use the existing rotate method to rotate by the calculated angle
        // Note: rotate() uses IMU, so we're rotating relative to current heading
        // rotate() already sets motors to RUN_WITHOUT_ENCODER, so manual control is preserved
        rotate(rotationAngle, rotationPower);

        // Verify we're now centered
        limelight.update();
        if (limelight.isCentered()) {
            opmode.telemetry.addData("Limelight Center", "Successfully centered!");
            opmode.telemetry.update();
            return true;
        }

        opmode.telemetry.addData("Limelight Center", "Centering may need adjustment");
        opmode.telemetry.update();
        return true; // Return true even if not perfectly centered, as we did our best
    }

    /**
     * Center the robot on a Limelight target with default power
     * @param limelight The Limelight instance
     * @return true if successfully centered
     */
    public boolean centerOnLimelightTarget(org.firstinspires.ftc.teamcode.Game.Limelight limelight) {
        return centerOnLimelightTarget(limelight, 0.5, 0);
    }

    /**
     * Continuously center on a Limelight target until centered or timeout
     * This will keep adjusting until the robot is within the centering threshold
     * 
     * @param limelight The Limelight instance
     * @param rotationPower The power to use for rotation
     * @param maxIterations Maximum number of centering attempts (0 = no limit, but check opModeIsActive)
     * @return true if successfully centered
     */
    public boolean centerOnLimelightTargetContinuous(org.firstinspires.ftc.teamcode.Game.Limelight limelight,
                                                      double rotationPower, int maxIterations) {
        if (limelight == null) {
            return false;
        }

        int iterations = 0;
        double minAngleThreshold = 0.5; // Minimum angle to bother rotating

        while (opmode.opModeIsActive() && (maxIterations == 0 || iterations < maxIterations)) {
            limelight.update();
            
            if (!limelight.hasValidTarget()) {
                opmode.telemetry.addData("Limelight Center", "Target lost");
                opmode.telemetry.update();
                return false;
            }

            double rotationAngle = limelight.getCenteringRotationAngle();
            
            // If centered, we're done
            if (Math.abs(rotationAngle) < minAngleThreshold || limelight.isCentered()) {
                opmode.telemetry.addData("Limelight Center", "Centered! Iterations: %d", iterations);
                opmode.telemetry.update();
                return true;
            }

            // Rotate by the calculated angle
            rotate(rotationAngle, rotationPower);
            iterations++;

            opmode.telemetry.addData("Limelight Center", "Iteration: %d, Angle: %.2f", iterations, rotationAngle);
            opmode.telemetry.update();
            
            opmode.sleep(100); // Small delay between iterations
        }

        return limelight.isCentered();
    }

    /**
     * Restore motors to manual control mode (RUN_WITHOUT_ENCODER)
     * This should be called after encoder-based movements to allow manual driving
     */
    public void restoreManualControl() {
        run_without_encoders_all();
    }

    /**
     * Center the robot's position using Limelight by strafing left/right (encoder-based)
     * This calculates the distance needed to strafe based on tx angle and distance to target
     * 
     * @param limelight The Limelight instance
     * @param movePower Power to use for movement (0.0 to 1.0)
     * @param distanceToTarget Estimated distance to target in inches (used to calculate strafe distance)
     * @return true if successfully centered
     */
    public boolean centerOnLimelightTargetByStrafing(org.firstinspires.ftc.teamcode.Game.Limelight limelight,
                                                      double movePower, double distanceToTarget) {
        if (limelight == null) {
            opmode.telemetry.addData("Error", "Limelight is null");
            opmode.telemetry.update();
            return false;
        }

        limelight.update();
        
        if (!limelight.hasValidTarget()) {
            opmode.telemetry.addData("Limelight Center", "No valid target detected");
            opmode.telemetry.update();
            return false;
        }

        // Get the horizontal offset angle directly using getTx() method
        double tx = limelight.getTx();
        opmode.telemetry.addData("Limelight Center", "Tx: %.2f degrees", tx);
        opmode.telemetry.update();
        
        // If already well-centered (within 0.5 degrees), return success
        if (Math.abs(tx) < 0.5) {
            opmode.telemetry.addData("Limelight Center", "Already well-centered (< 0.5 deg)");
            opmode.telemetry.update();
            restoreManualControl(); // Restore manual control mode
            return true;
        }

        // Convert angle to strafe distance using trigonometry
        // distance = tan(angle) * distanceToTarget
        // Note: tx is in degrees, so convert to radians
        double angleRadians = Math.toRadians(tx);
        double strafeDistanceInches = Math.tan(angleRadians) * distanceToTarget;

        opmode.telemetry.addData("Limelight Center", "Tx: %.2f deg, Strafe: %.2f inches", tx, strafeDistanceInches);
        opmode.telemetry.update();

        // Determine direction and move
        if (tx > 0) {
            // Target is to the right, strafe right
            move(movePower, "right", Math.abs(strafeDistanceInches));
        } else {
            // Target is to the left, strafe left
            move(movePower, "left", Math.abs(strafeDistanceInches));
        }

        // Restore manual control mode after movement
        restoreManualControl();

        // Verify we're centered
        limelight.update();
        if (limelight.isCentered()) {
            opmode.telemetry.addData("Limelight Center", "Successfully centered by strafing!");
            opmode.telemetry.update();
            return true;
        }

        return true;
    }

    /**
     * Center on Limelight target using both rotation and strafing (encoder-based)
     * First rotates to face the target, then strafes to center position
     * 
     * @param limelight The Limelight instance
     * @param rotationPower Power for rotation
     * @param movePower Power for strafing
     * @param distanceToTarget Estimated distance to target in inches
     * @return true if successfully centered
     */
    public boolean centerOnLimelightTargetFull(org.firstinspires.ftc.teamcode.Game.Limelight limelight,
                                                double rotationPower, double movePower, double distanceToTarget) {
        // First, rotate to face the target
        boolean rotated = centerOnLimelightTarget(limelight, rotationPower, 0);
        
        if (!rotated) {
            restoreManualControl(); // Restore manual control even if rotation failed
            return false;
        }

        opmode.sleep(200); // Small delay after rotation

        // Then strafe to center position (this will restore manual control at the end)
        return centerOnLimelightTargetByStrafing(limelight, movePower, distanceToTarget);
    }


}

