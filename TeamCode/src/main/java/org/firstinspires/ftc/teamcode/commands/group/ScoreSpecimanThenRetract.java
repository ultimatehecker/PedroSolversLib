package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ElevatorController;
import org.firstinspires.ftc.teamcode.commands.ManipulatorController;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;

public class ScoreSpecimanThenRetract extends SequentialCommandGroup {
    public ScoreSpecimanThenRetract(Manipulator manipulator, Elevator elevator) {
        setName("Score Speciman Then Retract");
        addCommands(
                new ManipulatorController(manipulator, Manipulator.ManipulatorState.SPECIMAN_SCORE, false),
                new ElevatorController(elevator, Elevator.ElevatorState.SPECIMAN_HIGH_SCORE),
                new ManipulatorController(manipulator, manipulator.getState(), true).alongWith(new WaitCommand(300)),
                new ElevatorController(elevator, Elevator.ElevatorState.RETRACTED).alongWith(new ManipulatorController(manipulator, Manipulator.ManipulatorState.TRANSFER, true))
        );
    }
}
