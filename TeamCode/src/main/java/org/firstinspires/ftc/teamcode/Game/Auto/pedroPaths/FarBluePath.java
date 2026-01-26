package org.firstinspires.ftc.teamcode.Game.Auto.pedroPaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Game.Subsystems.Intake;

@Autonomous(name = "Pedro Autonomous Far Blue", group = "Autonomous")
@Configurable
public class FarBluePath extends OpMode {

    private static final int RUN_TO_FIRST = 0;
    private static final int INTAKE_FIRST = 1;
    private static final int BACK_TO_SHOOT_FIRST = 2;
    private static final int TURN_TO_SHOOT_FIRST = 3;
    private static final int SHOOT_FIRST = 4;

    private static final int RUN_TO_SECOND = 5;
    private static final int INTAKE_SECOND = 6;
    private static final int BACK_TO_SHOOT_SECOND = 7;
    private static final int SHOOT_SECOND = 8;

    private static final int RUN_TO_THIRD = 9;
    private static final int INTAKE_THIRD = 10;
    private static final int BACK_TO_SHOOT_THIRD = 11;
    private static final int SHOOT_THIRD = 12;

    private static final int LEAVE = 13;
    private static final int DONE = 14;

    private static final double SHOOT_RPM = 3200;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = DONE;
    private Paths paths;

    private Intake intake;
    private SafeOuttake outtake; // <- use safe outtake so it doesn't crash on missing kickerPosition

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap);
        outtake = new SafeOuttake(hardwareMap, panelsTelemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60.000, 10.000, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public final PathChain runToFirstIntakePos;
        public final PathChain intakeFirst;
        public final PathChain goBackToShootingFirst;
        public final PathChain turnToShootFirst;

        public final PathChain runToSecondIntakePos;
        public final PathChain intakeSecond;
        public final PathChain goBackToShootingSecond;

        public final PathChain runToThirdIntakePos;
        public final PathChain intakeThird;
        public final PathChain goBackToShootingThird;

        public final PathChain leave;

        public Paths(Follower follower) {

            runToFirstIntakePos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(45.000, 36.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(45.000, 36.000), new Pose(15.000, 36.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingFirst = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.000, 36.000), new Pose(60.000, 10.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            turnToShootFirst = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(60.000, 10.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(110))
                    .build();

            runToSecondIntakePos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(41.849, 59.597)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(41.849, 59.597), new Pose(20.219, 59.361)))
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingSecond = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.219, 59.361), new Pose(60.000, 10.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(109))
                    .build();

            runToThirdIntakePos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(42.554, 83.930)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intakeThird = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(42.554, 83.930), new Pose(18.691, 83.812)))
                    .setTangentHeadingInterpolation()
                    .build();

            goBackToShootingThird = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.691, 83.812), new Pose(60.000, 10.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(110))
                    .build();

            leave = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 10.000), new Pose(55.132, 54.424)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {

        if (pathState == DONE) {
            follower.followPath(paths.runToFirstIntakePos);
            return RUN_TO_FIRST;
        }

        if (!follower.isBusy()) {
            switch (pathState) {

                case RUN_TO_FIRST:
                    intake.intakeOn();
                    follower.followPath(paths.intakeFirst);
                    pathState = INTAKE_FIRST;
                    break;

                case INTAKE_FIRST:
                    intake.intakeOff();
                    follower.followPath(paths.goBackToShootingFirst);
                    pathState = BACK_TO_SHOOT_FIRST;
                    break;

                case BACK_TO_SHOOT_FIRST:
                    follower.followPath(paths.turnToShootFirst);
                    pathState = TURN_TO_SHOOT_FIRST;
                    break;

                case TURN_TO_SHOOT_FIRST:
                    outtake.resetKickerCycle();
                    pathState = SHOOT_FIRST;
                    break;

                case SHOOT_FIRST:
                    outtake.setRPM(SHOOT_RPM);
                    outtake.enableKickerCycle(true, SHOOT_RPM);
                    if (outtake.getKickerCycleCount() >= 3) {
                        outtake.setRPM(0);
                        outtake.resetKickerCycle();
                        follower.followPath(paths.runToSecondIntakePos);
                        pathState = RUN_TO_SECOND;
                    }
                    break;

                case RUN_TO_SECOND:
                    intake.intakeOn();
                    follower.followPath(paths.intakeSecond);
                    pathState = INTAKE_SECOND;
                    break;

                case INTAKE_SECOND:
                    intake.intakeOff();
                    follower.followPath(paths.goBackToShootingSecond);
                    pathState = BACK_TO_SHOOT_SECOND;
                    break;

                case BACK_TO_SHOOT_SECOND:
                    outtake.resetKickerCycle();
                    pathState = SHOOT_SECOND;
                    break;

                case SHOOT_SECOND:
                    outtake.setRPM(SHOOT_RPM);
                    outtake.enableKickerCycle(true, SHOOT_RPM);
                    if (outtake.getKickerCycleCount() >= 3) {
                        outtake.setRPM(0);
                        outtake.resetKickerCycle();
                        follower.followPath(paths.runToThirdIntakePos);
                        pathState = RUN_TO_THIRD;
                    }
                    break;

                case RUN_TO_THIRD:
                    intake.intakeOn();
                    follower.followPath(paths.intakeThird);
                    pathState = INTAKE_THIRD;
                    break;

                case INTAKE_THIRD:
                    intake.intakeOff();
                    follower.followPath(paths.goBackToShootingThird);
                    pathState = BACK_TO_SHOOT_THIRD;
                    break;

                case BACK_TO_SHOOT_THIRD:
                    outtake.resetKickerCycle();
                    pathState = SHOOT_THIRD;
                    break;

                case SHOOT_THIRD:
                    outtake.setRPM(SHOOT_RPM);
                    outtake.enableKickerCycle(true, SHOOT_RPM);
                    if (outtake.getKickerCycleCount() >= 3) {
                        outtake.setRPM(0);
                        outtake.resetKickerCycle();
                        follower.followPath(paths.leave);
                        pathState = LEAVE;
                    }
                    break;

                case LEAVE:
                    intake.intakeOff();
                    outtake.setRPM(0);
                    pathState = DONE;
                    requestOpModeStop();
                    break;

                default:
                    pathState = DONE;
                    break;
            }
        }

        return pathState;
    }

    /**
     * This is a "drop-in" safe version of your Outtake that won't crash
     * if the analog sensor "kickerPosition" is not configured.
     *
     * BUT: If your real Outtake has extra motors/servos etc, you should
     * copy the try/catch pattern into your real Outtake instead.
     */
    public static class SafeOuttake {

        private final TelemetryManager tel;
        private AnalogInput kickerPosition; // may be null

        private int kickerCycleCount = 0;
        private double rpmSetpoint = 0;

        public SafeOuttake(HardwareMap hardwareMap, TelemetryManager tel) {
            this.tel = tel;

            // ✅ Guarded hardwareMap access so OpMode doesn't die
            try {
                kickerPosition = hardwareMap.get(AnalogInput.class, "kickerPosition");
                tel.debug("Outtake", "kickerPosition sensor found");
            } catch (Exception e) {
                kickerPosition = null;
                tel.debug("Outtake", "kickerPosition NOT found (running without it)");
            }

            // TODO: If your real outtake maps motors/servos, do it here too
        }

        public void setRPM(double rpm) {
            rpmSetpoint = rpm;
            // TODO: set shooter motor velocity in your real outtake
        }

        public void enableKickerCycle(boolean enabled, double targetRPM) {
            // TODO: your real kicker logic goes here
            // This stub just increments count to match your state machine behavior.
            if (enabled && targetRPM > 0) {
                // If you have a sensor, you would use it here safely:
                if (kickerPosition != null) {
                    double voltage = kickerPosition.getVoltage();
                    // use voltage if needed
                }
                // fake "shots" so your auto progresses (REMOVE if using real outtake)
                kickerCycleCount++;
            }
        }

        public void resetKickerCycle() {
            kickerCycleCount = 0;
        }

        public int getKickerCycleCount() {
            return kickerCycleCount;
        }
    }
}
