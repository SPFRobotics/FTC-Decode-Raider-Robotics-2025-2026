package org.firstinspires.ftc.teamcode.pedroPaths;

import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

public class AutoFarBlueOutline extends OpMode {

    private PathChain autoPath;

    @Override
    public void init() {
        // Pick a nice visible starting point on Panels
        follower.setStartingPose(new Pose(72, 72, 0));
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Tracing Auto Far Blue path outline only.");
        telemetryM.debug("No shooting, no mechanisms.");
        telemetryM.update(telemetry);

        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {

        Pose start = new Pose(72, 72, 0);

        // === Converted outline of your mecanum auto ===
        Pose p1  = new Pose(72, 92, 0);               // forward 20
        Pose p2  = new Pose(102, 92, Math.PI / 2);    // strafe right 30
        Pose p3  = new Pose(72, 92, Math.PI / 2);     // strafe back
        Pose p4  = new Pose(72, 159, 0);              // forward 67

        Pose p5  = new Pose(52.1, 116.4, Math.toRadians(25)); // angled back 47
        Pose p6  = new Pose(82.1, 116.4, Math.toRadians(115)); // side jig
        Pose p7  = new Pose(52.1, 116.4, Math.toRadians(115));

        Pose p8  = new Pose(52.1, 163.4, Math.toRadians(25)); // forward 47

        Pose p9  = new Pose(39.9, 138.9, Math.toRadians(50)); // angled back 27
        Pose p10 = new Pose(69.9, 138.9, Math.toRadians(140));
        Pose p11 = new Pose(39.9, 138.9, Math.toRadians(140));

        Pose p12 = new Pose(39.9, 165.9, Math.toRadians(75)); // final forward 27

        autoPath = follower.pathBuilder()

                .addPath(new BezierLine(start, p1))
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierLine(p1, p2))
                .setConstantHeadingInterpolation(Math.PI / 2)

                .addPath(new BezierLine(p2, p3))
                .setConstantHeadingInterpolation(Math.PI / 2)

                .addPath(new BezierLine(p3, p4))
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierLine(p4, p5))
                .setLinearHeadingInterpolation(0, Math.toRadians(25))

                .addPath(new BezierLine(p5, p6))
                .setConstantHeadingInterpolation(Math.toRadians(115))

                .addPath(new BezierLine(p6, p7))
                .setConstantHeadingInterpolation(Math.toRadians(115))

                .addPath(new BezierLine(p7, p8))
                .setConstantHeadingInterpolation(Math.toRadians(25))

                .addPath(new BezierLine(p8, p9))
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(50))

                .addPath(new BezierLine(p9, p10))
                .setConstantHeadingInterpolation(Math.toRadians(140))

                .addPath(new BezierLine(p10, p11))
                .setConstantHeadingInterpolation(Math.toRadians(140))

                .addPath(new BezierLine(p11, p12))
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(75))

                .build();

        follower.followPath(autoPath);
    }

    @Override
    public void loop() {
        follower.update();
        draw();

        if (follower.atParametricEnd()) {
            telemetryM.debug("Path complete.");
            telemetryM.update(telemetry);
        }
    }
}
