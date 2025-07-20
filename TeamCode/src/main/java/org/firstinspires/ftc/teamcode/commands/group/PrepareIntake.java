package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.ElevatorController;
import org.firstinspires.ftc.teamcode.commands.IntakeController;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.utilities.constansts.ElevatorConstants;

public class PrepareIntake extends SequentialCommandGroup {
    public PrepareIntake(Intake intake, Elevator elevator) {
        setName("Prepare Intake");
        addCommands(
                new ConditionalCommand(
                        new ElevatorController(elevator, Elevator.ElevatorState.TRANSFER),
                        new WaitUntilCommand(() -> true),
                        () -> elevator.getPosition() < ElevatorConstants.transferHeight || elevator.getCorrectedVelocity() < 0 || elevator.isRetracted()
                ),
                new IntakeController(intake, Intake.IntakeState.HOVER_OUT, Intake.WristState.NORMAL, true),
                new WaitCommand(700),
                new ElevatorController(elevator, Elevator.ElevatorState.RETRACTED)
        );
    }
}
