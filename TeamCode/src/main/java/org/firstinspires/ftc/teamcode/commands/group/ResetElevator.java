package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ElevatorController;
import org.firstinspires.ftc.teamcode.commands.ManipulatorController;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;

public class ResetElevator extends SequentialCommandGroup {
    public ResetElevator(Manipulator manipulator, Elevator elevator) {
        setName("Reset Elevator");
        addCommands(
                new ManipulatorController(manipulator, Manipulator.ManipulatorState.TRANSFER, true),
                new ElevatorController(elevator, Elevator.ElevatorState.RETRACTED)
        );
    }

    public ResetElevator(Elevator elevator) {
        setName("Reset Elevator");
        addCommands(
                new ElevatorController(elevator, Elevator.ElevatorState.RETRACTED)
        );
    }
}
