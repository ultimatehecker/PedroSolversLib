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

        relativeLimelightOffset = new Transform2d(new Translation2d(4.963, 1.618), new Rotation2d());
        limelightFieldCoordinates = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        this.telemetryManager = telemetryManager;
    }

    public static class Detection {
        public final double x;
        public final double y;
        public final double distance;
        public final double targetAngle;

        public Detection(double x, double y, double distance, double targetAngle) {
            this.x = x;
            this.y = y;
            this.distance = distance;
            this.targetAngle = targetAngle;
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
            x = LimelightConstants.cameraHeightFromGround * Math.tan(Math.toRadians(result.getTy() + 90 - 20));
            y = Math.sqrt(y * y + LimelightConstants.cameraHeightFromGround * LimelightConstants.cameraHeightFromGround) * Math.tan(Math.toRadians(result.getTx()));

            targetAngle = Math.atan((x + LimelightConstants.distanceLimelight) / (y + LimelightConstants.lateralDistance));
            distance = Math.sqrt((x + LimelightConstants.lateralDistance) * (x + LimelightConstants.lateralDistance) + (y + LimelightConstants.distanceLimelight) * (y + LimelightConstants.distanceLimelight));

            detections.add(new Detection(x, y, distance, targetAngle));

            if(LimelightConstants.enableLogging) {
                telemetryManager.debug(
                        String.format("[LL] Some Position %d | X: %.2f, Y: %.2f, Distance: %.2f in | Target Angle: %.2f |",
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

    @Override
    public void periodic() {
        getLLResult();
        List<Detection> samples = getDetectionsRelative();
    }
}