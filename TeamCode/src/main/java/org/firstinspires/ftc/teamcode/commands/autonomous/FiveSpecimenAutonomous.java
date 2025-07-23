package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.pedropathing.pathgen.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autonomous.paths.Trajectories;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.group.PrepareSpeciman;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;

import java.util.function.BooleanSupplier;

public class FiveSpecimenAutonomous extends SequentialCommandGroup {
    private PathChain pathChain = Trajectories.fiveSpeciman;
    public FiveSpecimenAutonomous(Drivetrain drivetrain, Elevator elevator, Manipulator manipulator, Intake intake, BooleanSupplier isOpModeActive) {
        setName("Five Specimam Autonomous");
        addCommands(
                new WaitUntilCommand(isOpModeActive),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(0)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(1)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(2)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(3)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(4)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(5)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(6)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(7)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(8)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(9)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(10)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(11)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(12)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(13)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(14))

        );
    }
}
