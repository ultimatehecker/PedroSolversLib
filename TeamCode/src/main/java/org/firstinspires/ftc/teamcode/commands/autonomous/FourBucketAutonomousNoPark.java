package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autonomous.paths.Trajectories;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.BooleanSupplier;

public class FourBucketAutonomousNoPark extends SequentialCommandGroup {
    private final PathChain pathChain = Trajectories.fourBucketNoPark;
    private final BooleanSupplier isOpModeActive;

    public FourBucketAutonomousNoPark(Drivetrain drivetrain, Elevator elevator, Intake intake, BooleanSupplier isOpModeActive) {
        this.isOpModeActive = isOpModeActive;

        setName("Four Bucket No Park Autonomous");
        addCommands(
                new WaitUntilCommand(isOpModeActive),
                new FollowTrajectoryCommand(drivetrain, pathChain.getPath(0)).setCompletionThreshold(0.975)
        );
    }
}
