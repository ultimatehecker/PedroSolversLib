package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ElevatorController;
import org.firstinspires.ftc.teamcode.commands.ManipulatorController;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;

public class PrepareSpeciman extends SequentialCommandGroup {
    public PrepareSpeciman(Manipulator manipulator, Elevator elevator) {
        setName("Prepare Speciman");
        addCommands(
                new ManipulatorController(manipulator, Manipulator.ManipulatorState.SPECIMAN_READY, false),
                new ElevatorController(elevator, Elevator.ElevatorState.SPECIMAN_HIGH_READY)
        );
    }
}
