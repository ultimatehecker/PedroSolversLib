package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.teamcode.utilities.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utilities.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utilities.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.utilities.geometry.Translation2d;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Vision extends SubsystemBase {
    private Limelight3A limelight;

    private Pose2d limelightToTarget;
    private Pose2d limelightFieldCoordinates;
    private Transform2d relativeLimelightOffset;

    private AprilTagProcessor arducam;
    private VisionPortal visionPortal;

    private Position arducamPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles arducamOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private double distance;
    private double tx;
    private double ty;
    private double confidence;

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
        limelight.setPollRateHz(100);
        limelight.start();

        relativeLimelightOffset = new Transform2d(new Translation2d(4.963, 1.618), new Rotation2d());
        limelightFieldCoordinates = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        this.telemetryManager = telemetryManager;
    }

    public void computeCameraToSample() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            tx = llResult.getDetectorResults().get(0).getTargetXDegrees();
            ty = llResult.getDetectorResults().get(0).getTargetYDegrees();
            confidence = llResult.getDetectorResults().get(0).getConfidence();

            // X and Y are reversed because pedropathing uses a different coordinate system
            double x = LimelightConstants.cameraHeightFromGround * Math.tan(Math.toRadians(ty) + 90 - LimelightConstants.cameraAngleFromXAxis);
            double y = Math.sqrt(x * x + LimelightConstants.cameraHeightFromGround * LimelightConstants.cameraHeightFromGround) * (Math.tan(Math.toRadians(tx)));

            double theta = Math.atan((y + LimelightConstants.lateralDistance) / (x + LimelightConstants.distanceLimelight));
            distance = Math.sqrt((y + LimelightConstants.lateralDistance) * (y + LimelightConstants.lateralDistance) + (x + LimelightConstants.distanceLimelight) * (x + LimelightConstants.distanceLimelight));

            limelightToTarget = new Pose2d(x, y, Rotation2d.fromDegrees(theta));
        }
    }

    public void computeLimelightFieldCoordinates(Pose2d robotPose) {
        limelightFieldCoordinates = robotPose.transformBy(relativeLimelightOffset);

        //telemetryManager.debug("LL FCoords X: " + limelightFieldCoordinates.getX());
        //telemetryManager.debug("LL FCoords Y: " + limelightFieldCoordinates.getY());
        //telemetryManager.debug("LL FCoords θ: " + limelightFieldCoordinates.getRotation());
    }


    @Override
    public void periodic() {
        computeCameraToSample();

        telemetryManager.debug("LL FCoords X: " + limelightFieldCoordinates.getX());
        telemetryManager.debug("LL FCoords Y: " + limelightFieldCoordinates.getY());
        telemetryManager.debug("LL FCoords θ: " + limelightFieldCoordinates.getRotation());

        telemetryManager.debug("Detector Tx: " + tx);
        telemetryManager.debug("Detector Ty: " + ty);
        telemetryManager.debug("Detector Confidence: " + confidence);
    }
}
