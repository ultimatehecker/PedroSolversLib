package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ElevatorController;
import org.firstinspires.ftc.teamcode.commands.ManipulatorController;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;

public class PrepareSample extends ParallelCommandGroup {
    public PrepareSample (Manipulator manipulator, Elevator elevator) {
        setName("Prepare Sample");
        addCommands(
                new ElevatorController(elevator, Elevator.ElevatorState.SAMPLE_HIGH_SCORE),
                new WaitCommand(1500).andThen(new ManipulatorController(manipulator, Manipulator.ManipulatorState.SAMPLE_SCORE, false))
        );
    }
}
