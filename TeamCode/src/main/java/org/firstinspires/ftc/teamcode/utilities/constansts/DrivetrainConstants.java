package org.firstinspires.ftc.teamcode.utilities.constansts;

import org.firstinspires.ftc.teamcode.utilities.Units;
import org.firstinspires.ftc.teamcode.utilities.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utilities.geometry.Rotation2d;

public class DrivetrainConstants {
    public static final String fLMotorID = "leftFront";
    public static final String fRMotorID = "rightFront";
    public static final String bLMotorID = "leftRear";
    public static final String bRMotorID = "rightRear";

    public static final String imuID = "imu";

    public static final double trackWidth = 16.0; // in inches
    public static final double wheelBase = 16.0; // in inches

    // Starting Poses
    public static final Pose2d allBucketStartingPose = new Pose2d(wheelBase / 2, 104.000, Rotation2d.fromDegrees(0));
    public static final Pose2d preloadSpecimanStartingPose = new Pose2d(wheelBase / 2, 64.000, Rotation2d.fromDegrees(0));
    public static final Pose2d nonPreloadSpecimanStartingPose = new Pose2d(wheelBase / 2, 48.000, Rotation2d.fromDegrees(0));

    // Scoring Poses
    public static final Pose2d allBucketScoringPose = new Pose2d(16.500, 125.500, Rotation2d.fromDegrees(315));
}