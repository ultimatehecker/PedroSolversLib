package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ElevatorController;
import org.firstinspires.ftc.teamcode.commands.ManipulatorController;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;

public class ScoreSpecimen extends SequentialCommandGroup {
    public ScoreSpecimen(Manipulator manipulator, Elevator elevator) {
        setName("Score Specimen");
        addCommands(
                new ManipulatorController(manipulator, Manipulator.ManipulatorState.SPECIMAN_SCORE, false),
                new ElevatorController(elevator, Elevator.ElevatorState.SPECIMAN_HIGH_SCORE),
                new ManipulatorController(manipulator, manipulator.getState(), true).alongWith()
        );
    }
}
