package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class FiveSpecimenTraj {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                //Line 1
                .addPath(
                        new BezierLine(
                                new Point(8, 64, Point.CARTESIAN),
                                new Point(40, 76, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 2
                .addPath(
                        new BezierCurve(
                                new Point(40, 76, Point.CARTESIAN),
                                new Point(21,36.7, Point.CARTESIAN),
                                new Point(61.6, 36, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 3
                .addPath(
                        new BezierCurve(
                                new Point(61.6, 35, Point.CARTESIAN),
                                new Point(51, 24.7, Point.CARTESIAN),
                                new Point(24, 28, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 4
                .addPath(
                        new BezierLine(
                                new Point(24, 28, Point.CARTESIAN),
                                new Point(61.6, 26, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 5
                .addPath(
                        new BezierCurve(
                                new Point(61.6, 26, Point.CARTESIAN),
                                new Point(51, 16.6, Point.CARTESIAN),
                                new Point(24, 20, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 6
                .addPath(
                        new BezierLine(
                                new Point(24, 20, Point.CARTESIAN),
                                new Point(61.6, 17.3, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 7
                .addPath(
                        new BezierCurve(
                                new Point(61.6, 17.3, Point.CARTESIAN),
                                new Point(61.6, 14.3, Point.CARTESIAN),
                                new Point(24, 14.8, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 8
                .addPath(
                        new BezierLine(
                                new Point(24, 14.8, Point.CARTESIAN),
                                new Point(12, 20.8, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 9
                .addPath(
                        new BezierLine(
                                new Point(12, 20.8, Point.CARTESIAN),
                                new Point(40, 74, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 10
                .addPath(
                        new BezierLine(
                                new Point(40, 74, Point.CARTESIAN),
                                new Point(12, 20.8, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 11
                .addPath(
                        new BezierLine(
                                new Point(12, 20.8, Point.CARTESIAN),
                                new Point(40, 72, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 12
                .addPath(
                        new BezierLine(
                                new Point(40, 72, Point.CARTESIAN),
                                new Point(12, 20.8, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 13
                .addPath(
                        new BezierLine(
                                new Point(12, 20.8, Point.CARTESIAN),
                                new Point(40, 70, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 14
                .addPath(
                        new BezierLine(
                                new Point(40, 70, Point.CARTESIAN),
                                new Point(12, 20.8, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //Line 15
                .addPath(
                        new BezierLine(
                                new Point(12, 20.8, Point.CARTESIAN),
                                new Point(40, 68, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));

        return builder.build();
    }
}
