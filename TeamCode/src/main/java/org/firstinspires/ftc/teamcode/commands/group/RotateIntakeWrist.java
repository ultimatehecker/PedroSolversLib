package org.firstinspires.ftc.teamcode.commands.group;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.IntakeController;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.constansts.IntakeConstants;

public class RotateIntakeWrist extends SequentialCommandGroup {
    public RotateIntakeWrist(Intake intake, Intake.WristState wristState) {
        setName("Rotate Intake Wrist");
        addCommands(
                new ConditionalCommand(
                        new IntakeController(intake, intake.getIntakeState(), wristState, intake.isClawOpen()),
                        new WaitUntilCommand(() -> true),
                        () -> intake.getLinkagePosition() == IntakeConstants.intakeLinkageServoInPosition || intake.getArmPosition() == IntakeConstants.intakeArmIntakingPosition
                )
        );
    }
}
