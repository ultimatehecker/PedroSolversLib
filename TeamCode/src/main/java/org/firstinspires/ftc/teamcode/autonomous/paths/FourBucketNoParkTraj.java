package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.utilities.constansts.DrivetrainConstants;

public class FourBucketNoParkTraj {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath( // Starting Pose to Scoring Pose
                        new BezierLine(
                                new Point(8.000, 112.000, Point.CARTESIAN),
                                new Point(13.750, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .addPath( // Scoring Pose to Pickup Point 1
                        new BezierLine(
                                new Point(13.750, 130.000, Point.CARTESIAN),
                                new Point(24.250, 126.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-11.5))
                .addPath( // Pickup Point 1 to Scoring Pose
                        new BezierLine(
                                new Point(24.250, 126.250, Point.CARTESIAN),
                                new Point(13.750, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(315))
                .addPath( // Scoring Pose to Pickup Point 2
                        new BezierLine(
                                new Point(13.750, 130.000, Point.CARTESIAN),
                                new Point(23.50, 130.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .addPath( // Pickup Point 2  to Scoring Pose
                        new BezierLine(
                                new Point(23.50, 130.500, Point.CARTESIAN),
                                new Point(13.750, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .addPath( // Scoring Pose to Pickup Point 3
                        new BezierLine(
                                new Point(13.750, 130.000, Point.CARTESIAN),
                                new Point(30.750, 125.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(45)) // stop
                .addPath( // Pickup Point 3 to Scoring Pose
                        new BezierLine(
                                new Point(30.750, 125.250, Point.CARTESIAN),
                                new Point(13.750, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(315))
                .addPath( // Scoring Pose to Park
                        new BezierCurve(
                                new Point(13.750, 130.000, Point.CARTESIAN),
                                new Point(61.75, 110.000, Point.CARTESIAN),
                                new Point(60.000, 100.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation();

        return builder.build();
    }
}