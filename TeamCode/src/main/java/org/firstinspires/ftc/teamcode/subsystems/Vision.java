package org.firstinspires.ftc.teamcode.subsystems;

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

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
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

    private List<Pose2d> fieldTargets;
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
        limelight.setPollRateHz(20);
        limelight.start();

        relativeLimelightOffset = new Transform2d(new Translation2d(4.963, 1.618), new Rotation2d());
        limelightFieldCoordinates = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        fieldTargets = new ArrayList<Pose2d>();
        this.telemetryManager = telemetryManager;
    }

    private void computeCameraToSample() {
        LLResult llResult = limelight.getLatestResult();

        if (llResult == null || !llResult.isValid()) return;

        LLResultTypes.DetectorResult result = llResult.getDetectorResults().get(0);
        tx = result.getTargetXDegrees(); // horizontal angle
        ty = result.getTargetYDegrees(); // vertical angle
        confidence = result.getConfidence();

        // Constants
        double cameraHeight = LimelightConstants.cameraHeightFromGround; // in inches
        double targetHeight = LimelightConstants.targetHeightFromGround; // in inches
        // 👇 make pitch negative if camera is tilted downward
        double cameraPitch = -Math.abs(LimelightConstants.cameraAngleFromXAxis);

        // Vertical geometry
        double totalVerticalAngle = Math.toRadians(ty + cameraPitch);
        double heightDifference = targetHeight - cameraHeight;

        // Forward distance
        double x = heightDifference / Math.tan(totalVerticalAngle);

        // Lateral offset
        double y = Math.tan(Math.toRadians(tx)) * x;

        // Estimate target pose relative to camera
        limelightToTarget = new Pose2d(x, y, new Rotation2d(Math.toRadians(tx)));

        telemetryManager.debug("[LL] tx=" + tx + " ty=" + ty + " conf=" + confidence);
        telemetryManager.debug("[LL] cameraPitch=" + cameraPitch + " totalVertAngle=" + Math.toDegrees(totalVerticalAngle));
        telemetryManager.debug("[LL] relTarget=" + limelightToTarget);
    }


    public boolean isLLResultThere() {
        LLResult llResult = limelight.getLatestResult();
        return llResult != null && llResult.isValid();
    }

    public void computeFieldCoordinates(Pose2d robotPose) {
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) return;

        fieldTargets.clear();

        // Include Limelight's yaw
        Rotation2d cameraYaw = Rotation2d.fromDegrees(-30); // yaw to the right
        Transform2d limelightTransform = new Transform2d(
                relativeLimelightOffset.getTranslation(),
                relativeLimelightOffset.getRotation().plus(cameraYaw)
        );
        Pose2d limelightPose = robotPose.transformBy(limelightTransform);

        for (LLResultTypes.DetectorResult result : llResult.getDetectorResults()) {
            double tx = result.getTargetXDegrees();
            double ty = result.getTargetYDegrees();
            double conf = result.getConfidence();

            if (conf < 0.3) continue;

            // Geometry
            double cameraHeight = LimelightConstants.cameraHeightFromGround;
            double targetHeight = LimelightConstants.targetHeightFromGround;
            double cameraPitch = -Math.abs(LimelightConstants.cameraAngleFromXAxis);

            double totalVerticalAngle = Math.toRadians(ty + cameraPitch);
            double heightDifference = targetHeight - cameraHeight;

            double x = heightDifference / Math.tan(totalVerticalAngle);
            double y = Math.tan(Math.toRadians(tx)) * x;

            Pose2d relTarget = new Pose2d(x, y, Rotation2d.fromDegrees(tx));
            Pose2d fieldTarget = limelightPose.transformBy(
                    new Transform2d(relTarget.getTranslation(), relTarget.getRotation())
            );

            fieldTargets.add(fieldTarget);

            telemetryManager.debug("[LL] fieldTarget=" + fieldTarget + " conf=" + conf);
        }
    }

    public List<Pose2d> getSampleFieldCoordinates() {
        return fieldTargets;
    }


    @Override
    public void periodic() {
        computeCameraToSample();
        LLResult llResult = limelight.getLatestResult();

        telemetryManager.debug("Detector Tx: " + tx);
        telemetryManager.debug("Detector Ty: " + ty);
        telemetryManager.debug("Detector Confidence: " + confidence);
        telemetryManager.debug("Sample From Limelight: " + limelightToTarget);
        telemetryManager.debug("FieldTargets: " + getSampleFieldCoordinates());
    }
}
