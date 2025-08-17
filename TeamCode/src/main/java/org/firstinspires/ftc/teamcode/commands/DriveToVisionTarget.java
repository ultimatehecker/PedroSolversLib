package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.pedropathing.pathgen.BezierCurve;
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

import java.util.List;

public class DriveToVisionTarget extends CommandBase {
    private Drivetrain drivetrain;
    private Vision vision;
    private TelemetryManager telemetryManager;
    private Path trajectory;

    public DriveToVisionTarget(Drivetrain drivetrain, Vision vision, TelemetryManager telemetryManager) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.telemetryManager = telemetryManager;
    }

    @Override
    public void initialize() {
        drivetrain.setMovementVectors(0,0,0);
        drivetrain.follower.breakFollowing();

        List<Pose2d> targets = vision.getSampleFieldCoordinates();

        if (!vision.isLLResultThere()) {
            end(true);
            return;
        }

        Pose2d mainTarget = targets.get(0);
        Pose2d drivetrainPose = drivetrain.getPose();

        Transform2d drivetrainToClaw = new Transform2d(new Translation2d(24, 0), new Rotation2d(0));
        Pose2d clawLocation = drivetrainPose.transformBy(drivetrainToClaw);

        PathBuilder pathBuilder = new PathBuilder();
        pathBuilder.addPath(
                new BezierCurve(
                        new Point(clawLocation.getX(), clawLocation.getY(), Point.CARTESIAN),
                        new Point( (clawLocation.getX() + mainTarget.getX()) / 2,
                                (clawLocation.getY() + mainTarget.getY()) / 2,
                                Point.CARTESIAN),
                        new Point(mainTarget.getX(), mainTarget.getY(), Point.CARTESIAN)
                )
        ).setLinearHeadingInterpolation(drivetrainPose.getRotation().getRadians(), Math.toRadians(0));

        trajectory = pathBuilder.build().getPath(0);
        drivetrain.follower.followPath(trajectory);
        telemetryManager.debug("AutoIntake: Created trajectory from " + clawLocation + " to " + mainTarget);
    }

    @Override
    public void execute() {
        telemetryManager.debug("Trajectory: " + trajectory);
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.follower.isBusy();
    }
}
