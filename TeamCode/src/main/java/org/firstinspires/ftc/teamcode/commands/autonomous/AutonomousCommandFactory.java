package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

@FunctionalInterface
public interface AutonomousCommandFactory {
    SequentialCommandGroup create(Object... args);
}
