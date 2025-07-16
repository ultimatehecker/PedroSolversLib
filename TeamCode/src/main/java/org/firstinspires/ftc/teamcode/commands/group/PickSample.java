package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.IntakeController;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class PickSample extends SequentialCommandGroup {
    public PickSample(Intake intake) {
        setName("Pick Sample") ;
        addCommands(
                new IntakeController(intake, Intake.IntakeState.INTAKING_OUT, intake.getWristState(), true),
                new WaitCommand(300),
                new IntakeController(intake, intake.getIntakeState(), intake.getWristState(), false),
                new WaitCommand(300),
                new IntakeController(intake, Intake.IntakeState.HOVER_OUT, intake.getWristState(), false)
        );
    }
}
