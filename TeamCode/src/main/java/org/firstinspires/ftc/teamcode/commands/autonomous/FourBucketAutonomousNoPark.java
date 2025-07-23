package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.pedropathing.pathgen.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autonomous.paths.Trajectories;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.HoldPoseCommand;
import org.firstinspires.ftc.teamcode.commands.group.PickSample;
import org.firstinspires.ftc.teamcode.commands.group.PrepareIntake;
import org.firstinspires.ftc.teamcode.commands.group.PrepareSample;
import org.firstinspires.ftc.teamcode.commands.group.RetractIntakeAndTransfer;
import org.firstinspires.ftc.teamcode.commands.group.RotateIntakeWrist;
import org.firstinspires.ftc.teamcode.commands.group.ScoreSampleThenRetract;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;

import java.util.function.BooleanSupplier;

public class FourBucketAutonomousNoPark extends SequentialCommandGroup {
    private final PathChain pathChain = Trajectories.fourBucketNoPark;
    private final BooleanSupplier isOpModeActive;

    public FourBucketAutonomousNoPark(Drivetrain drivetrain, Elevator elevator, Manipulator manipulator, Intake intake, BooleanSupplier isOpModeActive) {
        this.isOpModeActive = isOpModeActive;

        setName("Four Bucket No Park Autonomous");
        addCommands(
                new WaitUntilCommand(isOpModeActive),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(0)).alongWith(new PrepareSample(manipulator, elevator)),
                new ScoreSampleThenRetract(manipulator, elevator).interruptOn(elevator::isRetracting),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(1)).alongWith(new PrepareIntake(intake, elevator, Intake.WristState.DEG30)),
                new PickSample(intake).raceWith(new WaitCommand(300)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(2)).alongWith(new RetractIntakeAndTransfer(elevator, intake, manipulator)),
                new PrepareSample(manipulator, elevator).alongWith(new WaitCommand(400).andThen(new PrepareIntake(intake, Intake.WristState.NORMAL))),
                new ScoreSampleThenRetract(manipulator, elevator).interruptOn(elevator::isRetracting),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(3), true).andThen(new WaitCommand(400)),
                new PickSample(intake).raceWith(new WaitCommand(300)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(4)).alongWith(new RetractIntakeAndTransfer(elevator, intake, manipulator)),
                new PrepareSample(manipulator, elevator).alongWith(new WaitCommand(400).andThen(new PrepareIntake(intake, Intake.WristState.NORMAL))),
                new ScoreSampleThenRetract(manipulator, elevator).interruptOn(elevator::isRetracting),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(5), true).andThen(new WaitCommand(400)),
                new PickSample(intake).raceWith(new WaitCommand(400)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(6)).alongWith(new RetractIntakeAndTransfer(elevator, intake, manipulator)),
                new PrepareSample(manipulator, elevator),
                new ScoreSampleThenRetract(manipulator, elevator).interruptOn(elevator::isRetracting),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(7))
        );
    }
}
