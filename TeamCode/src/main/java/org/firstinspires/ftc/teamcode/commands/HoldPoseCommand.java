package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierPoint;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class HoldPoseCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Pose pose;
    private final boolean isFieldCentric;

    public HoldPoseCommand(Drivetrain drivetrain, Pose pose, boolean isFieldCentric) {
        this.drivetrain = drivetrain;
        this.pose = pose;
        this.isFieldCentric = isFieldCentric;
    }

    public HoldPoseCommand(Drivetrain drivetrain, Pose pose, double heading, boolean isFieldCentric) {
        this.drivetrain = drivetrain;
        this.pose = pose;
        this.isFieldCentric = isFieldCentric;
    }

    public HoldPoseCommand(Drivetrain drivetrain, BezierPoint bezierPoint, double heading, boolean isFieldCentric) {
        this.drivetrain = drivetrain;
        this.pose = new Pose(bezierPoint.getFirstControlPoint().getX(), bezierPoint.getFirstControlPoint().getY(), heading);
        this.isFieldCentric = isFieldCentric;
    }

    @Override
    public void initialize() {
        if (!isFieldCentric) {
            //pose.add(drivetrain.follower.getPose());
        }

        drivetrain.follower.holdPoint(pose);
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.follower.breakFollowing();
    }
}