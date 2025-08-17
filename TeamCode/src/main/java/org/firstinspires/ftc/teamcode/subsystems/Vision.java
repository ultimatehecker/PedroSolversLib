package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;
import android.util.Size;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

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

        relativeLimelightOffset = new Transform2d(new Translation2d(4.963, 1.618), new Rotation2d());
        limelightFieldCoordinates = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        this.telemetryManager = telemetryManager;
    }

    public static class Detection {
        public final Pose2d pose;
        public final double confidence;

        public Detection(Pose2d pose, double confidence) {
            this.pose = pose;
            this.confidence = confidence;
        }
    }

    private void getLLResult() {
        limelightResult = limelight.getLatestResult();
    }

    @SuppressLint("DefaultLocale")
    public List<Detection> getDetectionsRelative() {
        LLResult result = limelightResult;
        if (result == null || !result.isValid()) return Collections.emptyList();

        List<Detection> detections = new ArrayList<>();
        int i = 0;

        for (LLResultTypes.DetectorResult d : result.getDetectorResults()) {
            double txDeg = d.getTargetXDegrees();
            double tyDeg = d.getTargetYDegrees();
            double confidence = d.getConfidence();

            if (confidence < 0.5) continue;

            double tx = Math.toRadians(txDeg);
            double ty = Math.toRadians(tyDeg);

            // Vertical geometry
            double totalPitch = -20 + ty;
            double heightDiff = LimelightConstants.targetHeightFromGround - LimelightConstants.cameraHeightFromGround;

            double forward = heightDiff / Math.tan(totalPitch);
            double sideways = Math.tan(tx) * forward;

            Pose2d relPose = new Pose2d(forward, sideways, new Rotation2d(tx));
            detections.add(new Detection(relPose, confidence));

            if(LimelightConstants.enableLogging) {
                telemetryManager.debug(
                        String.format("[LL] Relative Pose %d | X: %.2f, Y: %.2f, θ: %.2f rad (%.2f deg) | TX: %.2f, TY: %.2f |",
                                i,
                                relPose.getTranslation().getX(),
                                relPose.getTranslation().getY(),
                                relPose.getRotation().getRadians(),
                                relPose.getRotation().getDegrees(),
                                tx,
                                ty
                        )
                );
            }

            i++;
        }

        // Sort from high to low confidence
        detections.sort((a, b) -> Double.compare(b.confidence, a.confidence));
        return detections;
    }

    @SuppressLint("DefaultLocale")
    public List<Detection> getDetectionsField(Pose2d robotPose) {
        List<Detection> relativeDetections = getDetectionsRelative();
        if (relativeDetections.isEmpty()) return relativeDetections;

        // Build camera pose from robot pose
        Rotation2d cameraYaw = Rotation2d.fromDegrees(-30); // yaw right maybe???
        Transform2d cameraTransform = new Transform2d(
                relativeLimelightOffset.getTranslation(),
                relativeLimelightOffset.getRotation().plus(cameraYaw)
        );

        Pose2d cameraPose = robotPose.transformBy(cameraTransform);
        List<Detection> fieldDetections = new ArrayList<>();
        int i = 0;

        for (Detection d : relativeDetections) {
            Pose2d fieldPose = cameraPose.transformBy(
                    new Transform2d(d.pose.getTranslation(), d.pose.getRotation())
            );

            if(LimelightConstants.enableLogging) {
                telemetryManager.debug(
                        String.format("[LL] Field Pose %d | X: %.2f, Y: %.2f, θ: %.2f rad (%.2f deg) | Confidence: %.2f",
                                i,
                                fieldPose.getTranslation().getX(),
                                fieldPose.getTranslation().getY(),
                                fieldPose.getRotation().getRadians(),
                                fieldPose.getRotation().getDegrees(),
                                d.confidence
                        )
                );
            }

            fieldDetections.add(new Detection(fieldPose, d.confidence));
            i++;
        }

        if(LimelightConstants.enableLogging) {
            telemetryManager.debug(
                    String.format("[LL] Camera Pose | X: %.2f, Y: %.2f, θ: %.2f rad (%.2f deg) |",
                            cameraPose.getTranslation().getX(),
                            cameraPose.getTranslation().getY(),
                            cameraPose.getRotation().getRadians(),
                            cameraPose.getRotation().getDegrees()
                    )
            );
        }

        return fieldDetections;
    }


    @Override
    public void periodic() {
        getLLResult();
    }
}