package org.firstinspires.ftc.teamcode.autonomous.paths;

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
                .addPath(
                        new BezierLine(
                                new Point(8.000, 112.000, Point.CARTESIAN),
                                new Point(13.750, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addPath(
                        new BezierLine(
                                new Point(13.750, 130.000, Point.CARTESIAN),
                                new Point(27.000, 120.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(27.000, 121.000, Point.CARTESIAN),
                                new Point(6.500, 129.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        new BezierLine(
                                new Point(6.500, 129.000, Point.CARTESIAN),
                                new Point(27.000, 130.100, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(27.000, 130.100, Point.CARTESIAN),
                                new Point(6.000, 128.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        new BezierLine(
                                new Point(6.000, 128.000, Point.CARTESIAN),
                                new Point(37.000, 128.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        new BezierLine(
                                new Point(37.000, 128.000, Point.CARTESIAN),
                                new Point(5.500, 128.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        new BezierLine(
                                new Point(5.500, 128.000, Point.CARTESIAN),
                                new Point(6.000, 106.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        new BezierLine(
                                new Point(6.000, 106.000, Point.CARTESIAN),
                                new Point(5.500, 128.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        new BezierLine(
                                new Point(5.500, 128.000, Point.CARTESIAN),
                                new Point(20.000, 120.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));

        return builder.build();
    }
}