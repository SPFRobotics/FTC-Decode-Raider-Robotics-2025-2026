package org.firstinspires.ftc.teamcode.Testing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Resources.PedroPathing.Constants;

@TeleOp(name = "Pedro Simple Test", group = "Testing")
public class PedroSimpleTest extends OpMode {

    private Follower follower;
    private Turret turret;
    private Pose currentPose;
    //private boolean fieldCentric = false;
    ElapsedTime loopTime;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        loopTime = new ElapsedTime();


        turret = new Turret(hardwareMap, true);                                      
    }

    @Override
    public void start() {
        currentPose = follower.getPose();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        loopTime.reset();
        follower.update();
        currentPose = follower.getPose();

        double speedFactor = (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) ? 0.5 : 1.0;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speedFactor,
                gamepad1.left_stick_x * speedFactor,
                gamepad1.right_stick_x * speedFactor,
                true
        );

        turret.aimAtGoal(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("Pose", "x=%.1f y=%.1f heading=%.1f°",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("Turret Target", turret.getTargetPosition());
        telemetry.addData("Turret At Target", turret.isTurretAtTarget());
        telemetry.addData("Loop Time", loopTime.milliseconds());


        telemetry.update();
    }
}
