package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.PrintCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.ElevatorController;
import org.firstinspires.ftc.teamcode.commands.IntakeController;
import org.firstinspires.ftc.teamcode.commands.ManipulatorController;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.utilities.constansts.ElevatorConstants;

public class RetractIntakeAndTransfer extends SequentialCommandGroup {
    public RetractIntakeAndTransfer(Elevator elevator, Intake intake, Manipulator manipulator) {
        setName("Retract Intake And Transfer");
        addCommands(
                new ConditionalCommand(
                        new ElevatorController(elevator, Elevator.ElevatorState.TRANSFER),
                        new PrintCommand("Done, Continuing"),
                        () -> elevator.getPosition() < 300
                ),
                new IntakeController(intake, Intake.IntakeState.TRANSFER, Intake.WristState.NORMAL, false),
                new WaitCommand(900),
                new ElevatorController(elevator, Elevator.ElevatorState.RETRACTED).alongWith(new InstantCommand(() -> manipulator.setClawOpen(true))),
                new ConditionalCommand(
                        new ManipulatorController(manipulator, Manipulator.ManipulatorState.TRANSFER, true).andThen(new WaitCommand(4000)),
                        new PrintCommand("Done, Continuing"),
                        () -> manipulator.getState() != Manipulator.ManipulatorState.TRANSFER
                ),
                new ManipulatorController(manipulator, manipulator.getState(), false),
                new WaitCommand(50).andThen(new IntakeController(intake, intake.getIntakeState(), intake.getWristState(), true))
        );
    }
}
