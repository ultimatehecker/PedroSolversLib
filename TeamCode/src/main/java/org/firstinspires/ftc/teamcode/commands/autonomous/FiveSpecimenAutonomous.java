package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.pedropathing.pathgen.PathChain;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autonomous.paths.Trajectories;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.ManipulatorController;
import org.firstinspires.ftc.teamcode.commands.group.PrepareSpeciman;
import org.firstinspires.ftc.teamcode.commands.group.ResetElevator;
import org.firstinspires.ftc.teamcode.commands.group.ScoreSpecimanThenRetract;
import org.firstinspires.ftc.teamcode.commands.group.ScoreSpecimen;
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
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(0)).alongWith(new PrepareSpeciman(manipulator, elevator)),
                new ScoreSpecimen(manipulator, elevator),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(1)).alongWith(new ResetElevator(manipulator, elevator)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(2)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(3)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(4)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(5)),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drivetrain, pathChain.getPath(6)),
                        new ManipulatorController(manipulator, Manipulator.ManipulatorState.WALL_INTAKE, true)
                ),

                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(7)),
                new InstantCommand(() -> manipulator.setClawOpen(false)).andThen(new WaitCommand(75)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(8)).alongWith(new PrepareSpeciman(manipulator, elevator)),
                new ScoreSpecimen(manipulator, elevator),
                new ParallelCommandGroup(
                        new ResetElevator(elevator),
                        new ManipulatorController(manipulator, Manipulator.ManipulatorState.WALL_INTAKE, true),
                        new FollowTrajectoryCommand(drivetrain, pathChain.getPath(9))
                ),
                new InstantCommand(() -> manipulator.setClawOpen(false)).andThen(new WaitCommand(75)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(10)).alongWith(new PrepareSpeciman(manipulator, elevator)),
                new ScoreSpecimen(manipulator, elevator),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drivetrain, pathChain.getPath(11)),
                        new ResetElevator(elevator),
                        new ManipulatorController(manipulator, Manipulator.ManipulatorState.WALL_INTAKE, true)
                ),
                new InstantCommand(() -> manipulator.setClawOpen(false)).andThen(new WaitCommand(75)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(12)).alongWith(new PrepareSpeciman(manipulator, elevator)),
                new ScoreSpecimen(manipulator, elevator),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(drivetrain, pathChain.getPath(13)),
                        new ResetElevator(elevator),
                        new ManipulatorController(manipulator, Manipulator.ManipulatorState.WALL_INTAKE, true)
                ),
                new InstantCommand(() -> manipulator.setClawOpen(false)).andThen(new WaitCommand(75)),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(14)).alongWith(new PrepareSpeciman(manipulator, elevator)),
                new ScoreSpecimanThenRetract(manipulator, elevator)
        );
    }
}
