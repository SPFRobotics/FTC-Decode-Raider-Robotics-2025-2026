
package org.firstinspires.ftc.teamcode.pedroPaths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

    public final class BlueShortPath  {

        private BlueShortPath() {}

        // Start pose from your .pp export
        public static final Pose START = new Pose(40.0, 136.0, Math.toRadians(90.0));

        /**
         * Call: follower.followPath(BlueShortPath.build(follower));
         */
        public static PathChain build(Follower follower) {

            // Waypoints (endpoints of each segment)
            Pose p1  = new Pose(73.0,  76.0,  Math.toRadians(142.0));
            Pose p2  = new Pose(40.0,  85.0,  Math.toRadians(180.0));
            Pose p3  = new Pose(16.6,  85.0,  Math.toRadians(180.0));
            Pose p4  = new Pose(78.0,  72.0,  Math.toRadians(142.0));
            Pose p5  = new Pose(65.0,  58.0,  Math.toRadians(180.0));
            Pose p6  = new Pose(15.0,  60.0,  0.0);                 // tangential heading; pose heading not used
            Pose p7  = new Pose(78.0,  72.0,  Math.toRadians(142.0));
            Pose p8  = new Pose(60.0,  32.0,  Math.toRadians(180.0));
            Pose p9  = new Pose(13.0,  35.0,  0.0);                 // tangential heading; pose heading not used
            Pose p10 = new Pose(78.0,  72.0,  Math.toRadians(142.0));
            Pose p11 = new Pose(82.0,  54.0,  Math.toRadians(270.0));

            return follower.pathBuilder()

                    // 1) Run From Wall: START -> p1, linear 90 -> 142
                    .addPath(new BezierLine(START, p1))
                    .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(142.0))

                    // 2) Run to first Intake pos: p1 -> p2, linear 142 -> 180
                    .addPath(new BezierLine(p1, p2))
                    .setLinearHeadingInterpolation(Math.toRadians(142.0), Math.toRadians(180.0))

                    // 3) Intake: p2 -> p3, constant 180
                    .addPath(new BezierLine(p2, p3))
                    .setConstantHeadingInterpolation(Math.toRadians(180.0))

                    // 4) Go back to shooting pos: p3 -> p4, linear 180 -> 142
                    .addPath(new BezierLine(p3, p4))
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(142.0))

                    // 5) run to second intake pos: p4 -> p5, linear 142 -> 180
                    .addPath(new BezierLine(p4, p5))
                    .setLinearHeadingInterpolation(Math.toRadians(142.0), Math.toRadians(180.0))

                    // 6) intake: p5 -> p6, tangential
                    .addPath(new BezierLine(p5, p6))
                    .setTangentHeadingInterpolation()

                    // 7) Go back to shooting pos: p6 -> p7, linear 180 -> 142
                    .addPath(new BezierLine(p6, p7))
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(142.0))

                    // 8) Run to third intake pos: p7 -> p8, linear 142 -> 180
                    .addPath(new BezierLine(p7, p8))
                    .setLinearHeadingInterpolation(Math.toRadians(142.0), Math.toRadians(180.0))

                    // 9) intake: p8 -> p9, tangential
                    .addPath(new BezierLine(p8, p9))
                    .setTangentHeadingInterpolation()

                    // 10) go back to shooting pos: p9 -> p10, linear 180 -> 142
                    .addPath(new BezierLine(p9, p10))
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(142.0))

                    // 11) Leave: p10 -> p11, linear 142 -> 270
                    .addPath(new BezierLine(p10, p11))
                    .setLinearHeadingInterpolation(Math.toRadians(142.0), Math.toRadians(270.0))

                    .build();
        }
    }


