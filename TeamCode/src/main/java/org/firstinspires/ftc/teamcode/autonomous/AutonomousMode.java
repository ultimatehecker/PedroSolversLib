package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.pathgen.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autonomous.paths.Trajectories;
import org.firstinspires.ftc.teamcode.commands.autonomous.AutonomousCommandFactory;
import org.firstinspires.ftc.teamcode.commands.autonomous.FiveSpecimenAutonomous;
import org.firstinspires.ftc.teamcode.commands.autonomous.FourBucketAutonomousNoPark;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.utilities.constansts.DrivetrainConstants;
import org.firstinspires.ftc.library.geometry.Pose2d;

import java.util.function.BooleanSupplier;

public enum AutonomousMode {
    FOUR_BUCKET_NO_PARK(
            DrivetrainConstants.allBucketStartingPose,
            Trajectories.fourBucketNoPark,
            args -> new FourBucketAutonomousNoPark(
                    (Drivetrain) args[0],
                    (Elevator) args[1],
                    (Manipulator) args[2],
                    (Intake) args[3],
                    (BooleanSupplier) args[4]
            )
    ),
    FOUR_BUCKET_PARK(
            DrivetrainConstants.allBucketStartingPose,
            Trajectories.fourBucketNoPark,
            args -> new FourBucketAutonomousNoPark(
                    (Drivetrain) args[0],
                    (Elevator) args[1],
                    (Manipulator) args[2],
                    (Intake) args[3],
                    (BooleanSupplier) args[4]
            )
    ),
    ONE_SPECIMAN_FOUR_BUCKET(
            DrivetrainConstants.allBucketStartingPose,
            Trajectories.fourBucketNoPark,
            args -> new FourBucketAutonomousNoPark(
                    (Drivetrain) args[0],
                    (Elevator) args[1],
                    (Manipulator) args[2],
                    (Intake) args[3],
                    (BooleanSupplier) args[4]
            )
    ),
    FIVE_SPECIMAN_PARK(
            DrivetrainConstants.preloadSpecimanStartingPose,
            Trajectories.fiveSpeciman,
            args -> new FiveSpecimenAutonomous(
                    (Drivetrain) args[0],
                    (Elevator) args[1],
                    (Manipulator) args[2],
                    (Intake) args[3],
                    (BooleanSupplier) args[4]
            )
    ),
    FIVE_SPECIMAN_PARK_NO_PRELOAD(
            DrivetrainConstants.nonPreloadSpecimanStartingPose,
            Trajectories.fourBucketNoPark,
            args -> new FourBucketAutonomousNoPark(
                    (Drivetrain) args[0],
                    (Elevator) args[1],
                    (Manipulator) args[2],
                    (Intake) args[3],
                    (BooleanSupplier) args[4]
            )
    ),
    FIVE_SPECIMAN_ONE_BUCKET_PARK(
            DrivetrainConstants.preloadSpecimanStartingPose,
            Trajectories.fourBucketNoPark,
            args -> new FourBucketAutonomousNoPark(
                    (Drivetrain) args[0],
                    (Elevator) args[1],
                    (Manipulator) args[2],
                    (Intake) args[3],
                    (BooleanSupplier) args[4]
            )
    ),
    SIX_SPECIMAN_PARK(
            DrivetrainConstants.preloadSpecimanStartingPose,
            Trajectories.fourBucketNoPark,
            args -> new FourBucketAutonomousNoPark(
                    (Drivetrain) args[0],
                    (Elevator) args[1],
                    (Manipulator) args[2],
                    (Intake) args[3],
                    (BooleanSupplier) args[4]
            )
    );

    private final Pose2d startingPose;
    private final PathChain pathChain;
    private final AutonomousCommandFactory commandFactory;

    private AutonomousMode(Pose2d startingPose, PathChain pathChain, AutonomousCommandFactory commandFactory) {
        this.startingPose = startingPose;
        this.pathChain = pathChain;
        this.commandFactory = commandFactory;
    }

    public Pose2d getStartingPose() {
        return this.startingPose;
    }

    public PathChain getPathChain() {
        return this.pathChain;
    }

    public SequentialCommandGroup getCommand(Object... args) {
        return commandFactory.create(args);
    }
}
