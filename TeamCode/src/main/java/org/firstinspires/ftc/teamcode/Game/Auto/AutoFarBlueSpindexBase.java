package org.firstinspires.ftc.teamcode.Game.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Spindex;

@Autonomous(name = "Auto Far Blue Spindex (Encoders)", group = "Autonomous")
public class AutoFarBlueSpindexBase extends LinearOpMode {

    private DcMotor backLeft, backRight, frontLeft, frontRight;
    private IMU imu;

    private Intake intake;
    private Spindex spindex;
    private Outtake outtake;

    private static final double SHOOT_RPM = 3200;

    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_IN = 3.78;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_IN * Math.PI);

    private static final double STRAFE_MULT = 1.10;

    @Override
    public void runOpMode() {

        backLeft = hardwareMap.dcMotor.get("backLeftDrive");
        backRight = hardwareMap.dcMotor.get("backRightDrive");
        frontLeft = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRight = hardwareMap.dcMotor.get("frontRightDrive");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        intake = new Intake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        outtake = new Outtake(hardwareMap, true);

        waitForStart();
        if (!opModeIsActive()) return;

        spindex.setMode(false);
        spindex.setIndex(0);

        turnRelative(20.0, 0.25);
        shoot3();
        turnRelative(-20.0, 0.25);

        goToLine1();
        intake3OnLine();
        returnFromLine1ToShoot();
        turnRelative(20.0, 0.25);
        shoot3();
        turnRelative(-20.0, 0.25);

        goToLine2();
        intake3OnLine();
        returnFromLine2ToShoot();
        turnRelative(20.0, 0.25);
        shoot3();
        turnRelative(-20.0, 0.25);

        goToLine3();
        intake3OnLine();
        returnFromLine3ToShoot();
        turnRelative(20.0, 0.25);
        shoot3();
        turnRelative(-20.0, 0.25);

        leave();
    }

    private void shoot3() {
        outtake.resetKickerCycle();
        outtake.setRPM(SHOOT_RPM);

        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive() && t.seconds() < 6.0 && outtake.getKickerCycleCount() < 3) {
            outtake.enableSpindexKickerCycle(true, SHOOT_RPM);
            idle();
        }

        outtake.setRPM(0);
        outtake.resetKickerCycle();
        sleep(150);
    }

    private void intake3OnLine() {
        intake.intakeOn();

        driveForward(0.35, 28);
        spindex.addIndex();
        sleep(120);

        driveForward(0.35, 28);
        spindex.addIndex();
        sleep(120);

        driveForward(0.35, 28);
        spindex.addIndex();
        sleep(120);

        intake.intakeOff();

        driveForward(0.60, -84);
        sleep(100);
    }

    private void goToLine1() {
        driveForward(0.80, 20);
        strafe(0.80, -18);
        driveForward(0.75, 18);
    }

    private void returnFromLine1ToShoot() {
        driveForward(0.80, -18);
        strafe(0.80, 18);
        driveForward(0.80, -20);
    }

    private void goToLine2() {
        strafe(0.90, -40);
        driveForward(0.75, 18);
    }

    private void returnFromLine2ToShoot() {
        driveForward(0.80, -18);
        strafe(0.90, 40);
    }

    private void goToLine3() {
        strafe(0.90, -62);
        driveForward(0.75, 18);
    }

    private void returnFromLine3ToShoot() {
        driveForward(0.80, -18);
        strafe(0.90, 62);
    }

    private void leave() {
        strafe(0.90, -34);
        driveForward(0.90, 8);
        outtake.setRPM(0);
        intake.intakeOff();
        stopDrive();
    }

    private void driveForward(double power, double inches) {
        encoderMove(power, inches, inches, inches, inches);
    }

    private void strafe(double power, double inches) {
        double adj = inches * STRAFE_MULT;
        encoderMove(power, adj, -adj, -adj, adj);
    }

    private void encoderMove(double power, double flIn, double frIn, double blIn, double brIn) {
        int fl = frontLeft.getCurrentPosition();
        int fr = frontRight.getCurrentPosition();
        int bl = backLeft.getCurrentPosition();
        int br = backRight.getCurrentPosition();

        frontLeft.setTargetPosition(fl + (int) Math.round(flIn * TICKS_PER_INCH));
        frontRight.setTargetPosition(fr + (int) Math.round(frIn * TICKS_PER_INCH));
        backLeft.setTargetPosition(bl + (int) Math.round(blIn * TICKS_PER_INCH));
        backRight.setTargetPosition(br + (int) Math.round(brIn * TICKS_PER_INCH));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = Math.abs(power);
        frontLeft.setPower(p);
        frontRight.setPower(p);
        backLeft.setPower(p);
        backRight.setPower(p);

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            idle();
        }

        stopDrive();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sleep(120);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void turnRelative(double degrees, double maxPower) {
        double start = getYawDeg();
        double target = AngleUnit.normalizeDegrees(start + degrees);

        while (opModeIsActive()) {
            double current = getYawDeg();
            double error = AngleUnit.normalizeDegrees(target - current);

            if (Math.abs(error) <= 0.6) break;

            double kP = 0.04;
            double pwr = error * kP;
            if (pwr > maxPower) pwr = maxPower;
            if (pwr < -maxPower) pwr = -maxPower;

            double min = 0.20;
            if (Math.abs(pwr) < min) pwr = min * (pwr / Math.abs(pwr));

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backLeft.setPower(pwr);
            frontLeft.setPower(pwr);
            backRight.setPower(-pwr);
            frontRight.setPower(-pwr);

            idle();
        }

        stopDrive();
        sleep(120);
    }

    private double getYawDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
