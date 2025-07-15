package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utilities.constansts.DrivetrainConstants;

public class FourBucketNoParkTraj {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.allBucketStartingPose.getAsPedroPose(),
                                DrivetrainConstants.allBucketScoringPose.getAsPedroPose()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        new BezierLine(
                                DrivetrainConstants.allBucketScoringPose.getAsPedroPose(),
                                new Pose(27.000, 120.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(27.000, 121.000),
                                new Pose(6.500, 129.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        new BezierLine(
                                new Pose(6.500, 129.000),
                                new Pose(27.000, 130.100)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(27.000, 130.100),
                                new Pose(6.000, 128.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        new BezierLine(
                                new Pose(6.000, 128.000),
                                new Pose(37.000, 128.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        new BezierLine(
                                new Pose(37.000, 128.000),
                                new Pose(5.500, 128.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        new BezierLine(
                                new Pose(5.500, 128.000),
                                new Pose(6.000, 106.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        new BezierLine(
                                new Pose(6.000, 106.000),
                                new Pose(5.500, 128.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        new BezierLine(
                                new Pose(5.500, 128.000),
                                new Pose(20.000, 120.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));

        return builder.build();
    }
}