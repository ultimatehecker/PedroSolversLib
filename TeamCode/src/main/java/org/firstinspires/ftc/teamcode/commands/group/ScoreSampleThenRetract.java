package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ElevatorController;
import org.firstinspires.ftc.teamcode.commands.ManipulatorController;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;

public class ScoreSampleThenRetract extends SequentialCommandGroup {
    public ScoreSampleThenRetract(Manipulator manipulator, Elevator elevator) {
        setName("Score Speciman Then Retract");
        addCommands(
                new ManipulatorController(manipulator, manipulator.getState(),  true),
                new WaitCommand(500),
                new ManipulatorController(manipulator, Manipulator.ManipulatorState.TRANSFER, true),
                new WaitCommand(300),
                new ElevatorController(elevator, Elevator.ElevatorState.RETRACTED)
        );
    }
}
