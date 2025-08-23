package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;
import android.util.Size;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utilities.constansts.LimelightConstants;
import org.firstinspires.ftc.library.geometry.Pose2d;
import org.firstinspires.ftc.library.geometry.Rotation2d;
import org.firstinspires.ftc.library.geometry.Transform2d;
import org.firstinspires.ftc.library.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Vision extends SubsystemBase {
    private Limelight3A limelight;
    private LLResult limelightResult;

    private Pose2d limelightToTarget;
    private Pose2d limelightFieldCoordinates;
    private Transform2d relativeLimelightOffset;

    private double x;
    private double y;
    private double distance;
    private double targetAngle;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private static Vision instance = null;
    public static synchronized Vision getInstance(HardwareMap aHardwareMap, TelemetryManager telemetryManager) {
        if(instance == null) {
            instance = new Vision(aHardwareMap, telemetryManager);
        }

        return instance;
    }

    private Vision(HardwareMap aHardwareMap, TelemetryManager telemetryManager) {
        limelight = aHardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(20);
        limelight.start();

        relativeLimelightOffset = new Transform2d(new Translation2d(4, 11.1), new Rotation2d());
        limelightFieldCoordinates = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        this.telemetryManager = telemetryManager;
    }


    public class Detection {
        public final Pose2d pose;       // Relative pose (from camera → target)
        public final double distance;   // Distance in inches
        public final double angle;      // Angle in radians
        public final double confidence; // Confidence score (0–1), optional

        // Constructor without confidence
        public Detection(Pose2d pose, double distance, double angle) {
            this(pose, distance, angle, 1.0); // default confidence = 1
        }

        // Constructor with confidence
        public Detection(Pose2d pose, double distance, double angle, double confidence) {
            this.pose = pose;
            this.distance = distance;
            this.angle = angle;
            this.confidence = confidence;
        }

        @Override
        public String toString() {
            return String.format(
                    "Detection | Pose: (X: %.2f, Y: %.2f, θ: %.2f°) | Distance: %.2f in | Angle: %.2f° | Conf: %.2f",
                    pose.getX(),
                    pose.getY(),
                    Math.toDegrees(pose.getRotation().getRadians()),
                    distance,
                    Math.toDegrees(angle),
                    confidence
            );
        }
    }


            private void getLLResult() {
        limelightResult = limelight.getLatestResult();
    }

    /*

    @SuppressLint("DefaultLocale")
    public List<Detection> getDetectionsRelative() {
        LLResult result = limelightResult;
        if (result == null || !result.isValid()) return Collections.emptyList();

        List<Detection> detections = new ArrayList<>();
        int i = 0;

        for (LLResultTypes.DetectorResult d : result.getDetectorResults()) {
            x = LimelightConstants.cameraHeightFromGround * Math.tan(Math.toRadians(result.getTy() + 90 - 20));
            y = Math.sqrt(y * y + LimelightConstants.cameraHeightFromGround * LimelightConstants.cameraHeightFromGround) * Math.tan(Math.toRadians(result.getTx()));

            targetAngle = Math.atan((x + LimelightConstants.distanceLimelight) / (y + LimelightConstants.lateralDistance));
            distance = Math.sqrt((x + LimelightConstants.lateralDistance) * (x + LimelightConstants.lateralDistance) + (y + LimelightConstants.distanceLimelight) * (y + LimelightConstants.distanceLimelight));

            detections.add(new Detection(x, y, distance, targetAngle));

            if(LimelightConstants.enableLogging) {
                telemetryManager.debug(
                        String.format("[LL] Sample Relative Position %d | X: %.2f, Y: %.2f, Distance: %.2f in | Target Angle: %.2f |",
                                i,
                                x,
                                y,
                                distance,
                                targetAngle
                        )
                );
            }

            i++;
        }

        return detections;
    }

     */

    @SuppressLint("DefaultLocale")
    public List<Detection> getDetectionsRelative() {
        LLResult result = limelightResult;
        if (result == null || !result.isValid()) return Collections.emptyList();

        List<Detection> detections = new ArrayList<>();
        int i = 0;

        // Assume camera yaw offset in degrees (positive = rotated CCW relative to robot fwd)
        double cameraYawDeg = -30; // example, adjust to match your mounting
        Rotation2d cameraYaw = Rotation2d.fromDegrees(cameraYawDeg);

        for (LLResultTypes.DetectorResult d : result.getDetectorResults()) {
            double x = LimelightConstants.cameraHeightFromGround * Math.tan(Math.toRadians(result.getTy() + 90 - 20)); // vertical math
            double y = Math.sqrt(x * x + LimelightConstants.cameraHeightFromGround * LimelightConstants.cameraHeightFromGround) * Math.tan(Math.toRadians(result.getTx()));

            double targetAngle = Math.atan((x + LimelightConstants.distanceLimelight) / (y + LimelightConstants.lateralDistance));
            double distance = Math.sqrt((x + LimelightConstants.lateralDistance) * (x + LimelightConstants.lateralDistance) + (y + LimelightConstants.distanceLimelight) * (y + LimelightConstants.distanceLimelight));

            // Apply yaw correction: rotate (x,y) vector
            Translation2d raw = new Translation2d(x, y);
            Translation2d rotated = raw.rotateBy(cameraYaw);

            // Make pose relative to limelight
            Pose2d relPoseFromLL = new Pose2d(rotated, cameraYaw);

            // Shift to robot center by applying relativeLimelightOffset (translation only)
            Pose2d relPoseFromRobot = relPoseFromLL.transformBy(relativeLimelightOffset);

            // Build detection
            detections.add(new Detection(relPoseFromRobot, distance, targetAngle, d.getConfidence()));

            if (LimelightConstants.enableLogging) {
                telemetryManager.debug(
                        String.format("[LL] Sample Relative %d | LL->Target: (%.2f, %.2f) | Robot->Target: (%.2f, %.2f) | Dist: %.2f in | θ: %.2f° | Conf: %.2f",
                                i,
                                rotated.getX(), rotated.getY(),
                                relPoseFromRobot.getX(), relPoseFromRobot.getY(),
                                distance,
                                Math.toDegrees(targetAngle),
                                d.getConfidence()
                        )
                );
            }
            i++;
        }

        return detections;
    }

    @Override
    public void periodic() {
        getLLResult();
        List<Detection> thing = getDetectionsRelative();
    }
}