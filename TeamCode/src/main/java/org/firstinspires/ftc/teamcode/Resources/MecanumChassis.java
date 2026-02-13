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
    public double inch_convert(double inch) { return inch * (537.7 / (4.09449 * Math.PI)); }
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
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
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

    public boolean motorsAreBusy(){
        if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            return true;
        }
        return false;
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
           throw new RuntimeException("Move direction must be forward, backward, left, or right.");
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

    public void moveWLoop(double movePower, @NonNull String moveDirection, double moveDistance){
        stop_and_reset_encoders_all(); //Sets encoder count to 0
        run_using_encoders_all();
        if (moveDirection.equals("forward")) {
            //Tell each wheel to move a certain amount
            backLeft.setTargetPosition((int) inch_convert(moveDistance)); //Converts the
            backRight.setTargetPosition((int) inch_convert(moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance));
            frontRight.setTargetPosition((int) inch_convert(moveDistance));
            run_to_position_all();
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
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection.equals("right")) {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            backRight.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection.equals("left")) {
            backLeft.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            backRight.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            frontRight.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else {
            throw new RuntimeException("Move direction must be forward, backward, left, or right.");
        }
    }

    //Overloads the methods to accept a character instead of an entire String as a direction while maintaining compatibility with the previous programs
    public boolean move(double movePower, @NonNull char moveDirection, double moveDistance){ // added support for moving lift and wheels at the same time.
        stop_and_reset_encoders_all(); //Sets encoder count to 0
        run_using_encoders_all();
        if (moveDirection == 'F' || moveDirection == 'f') {
            //Tell each wheel to move a certain amount
            backLeft.setTargetPosition((int) inch_convert(moveDistance)); //Converts the
            backRight.setTargetPosition((int) inch_convert(moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance));
            frontRight.setTargetPosition((int) inch_convert(moveDistance));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection == 'B' || moveDirection == 'b') {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance));
            backRight.setTargetPosition((int) inch_convert(-moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection == 'R' || moveDirection == 'r') {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            backRight.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection == 'L' || moveDirection == 'l') {
            backLeft.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            backRight.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            frontRight.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else {
            throw new RuntimeException("Move direction must be forward, backward, left, or right.");
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

    public void moveWLoop(double movePower, @NonNull char moveDirection, double moveDistance){
        stop_and_reset_encoders_all(); //Sets encoder count to 0
        run_using_encoders_all();
        if (moveDirection == 'F' || moveDirection == 'f') {
            //Tell each wheel to move a certain amount
            backLeft.setTargetPosition((int) inch_convert(moveDistance)); //Converts the
            backRight.setTargetPosition((int) inch_convert(moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance));
            frontRight.setTargetPosition((int) inch_convert(moveDistance));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection == 'B' || moveDirection == 'b') {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance));
            backRight.setTargetPosition((int) inch_convert(-moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection == 'R' || moveDirection == 'r') {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            backRight.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection == 'L' || moveDirection == 'l') {
            backLeft.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            backRight.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance * strafeMult));
            frontRight.setTargetPosition((int) inch_convert(moveDistance * strafeMult));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else {
            throw new RuntimeException("Move direction must be forward, backward, left, or right.");
        }
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
        //opmode.sleep(500);
    }
    public void wiggle(){
        move(1,"forward",3);
        move(1,"backward",3);
        wiggleCount++;
    }

    public int getWiggleCount(){
        return wiggleCount;
    }

    /**
     * Restore motors to manual control mode (RUN_WITHOUT_ENCODER)
     * This should be called after encoder-based movements to allow manual driving
     */
    public void restoreManualControl() {
        run_without_encoders_all();
    }
}

