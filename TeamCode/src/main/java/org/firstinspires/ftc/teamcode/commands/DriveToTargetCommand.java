package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.library.geometry.Pose2d;
import org.firstinspires.ftc.library.geometry.Rotation2d;
import org.firstinspires.ftc.library.geometry.Transform2d;
import org.firstinspires.ftc.library.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utilities.constansts.LimelightConstants;

import java.util.List;

public class DriveToTargetCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Vision vision;
    private final TelemetryManager telemetryManager;
    private Path trajectory;

    public DriveToTargetCommand(Drivetrain drivetrain, Vision vision, TelemetryManager telemetryManager) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.telemetryManager = telemetryManager;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setMovementVectors(0, 0, 0);
        drivetrain.follower.breakFollowing();

        // Get field detections (sorted highest confidence first)
        List<Vision.Detection> detections = vision.getDetectionsField(drivetrain.getPose());
        if (detections.isEmpty()) {
            telemetryManager.debug("[DriveToTarget] No detections found.");
            cancel();
            return;
        }

        Vision.Detection target = detections.get(0); // highest confidence
        Pose2d robotPose = drivetrain.getPose();

        // Transform robot → claw offset (24 inches forward)
        Transform2d drivetrainToClaw = new Transform2d(
                new Translation2d(
                        Math.cos(robotPose.getRotation().getRadians()) * 24.0,
                        Math.sin(robotPose.getRotation().getRadians()) * 24.0
                ),
                new Rotation2d()
        );
        Pose2d clawPose = robotPose.transformBy(drivetrainToClaw);

        Pose2d targetPose = target.pose;

        // Compute offset target (24 inches back along robot-to-target vector)
        Translation2d direction = targetPose.getTranslation().minus(robotPose.getTranslation());
        Translation2d unitDir = direction.div(direction.getNorm()); // normalize
        Translation2d offsetTarget = targetPose.getTranslation().minus(unitDir.times(24.0));

        Pose2d adjustedTarget = new Pose2d(offsetTarget, targetPose.getRotation());

        // Build trajectory from claw → target
        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(
                new BezierLine(
                        new Point(clawPose.getX(), clawPose.getY(), Point.CARTESIAN),
                        new Point(adjustedTarget.getX(), adjustedTarget.getY(), Point.CARTESIAN)
                )
        );

        trajectory = pathBuilder.build().getPath(0);

        if (LimelightConstants.enableLogging) {
            telemetryManager.debug("[DriveToTarget] Target: " + adjustedTarget);
            telemetryManager.debug("[DriveToTarget] Claw Pose: " + clawPose);
            telemetryManager.debug("[DriveToTarget] Path Built: " + trajectory);
        }

        if (trajectory != null) {
            drivetrain.follower.followPath(trajectory);
            telemetryManager.debug("[DriveToTarget] Following trajectory...");
        }

        drivetrain.follower.update();
    }

    @Override
    public void execute() {
        drivetrain.follower.update();
    }

    @Override
    public boolean isFinished() {
        return drivetrain.follower.getCurrentTValue() >= 1;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setMovementVectors(0, 0, 0);
        if (interrupted) {
            telemetryManager.debug("[DriveToTarget] Command interrupted.");
        } else {
            telemetryManager.debug("[DriveToTarget] Target reached.");
        }
    }
}