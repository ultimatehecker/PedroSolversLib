package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Manipulator;

public class ManipulatorController extends CommandBase {
    private final Manipulator manipulator;
    private Manipulator.ManipulatorState manipulatorState;
    private boolean clawOpen;

    public ManipulatorController(Manipulator manipulator, Manipulator.ManipulatorState manipulatorState, boolean clawOpen) {
        this.manipulator = manipulator;
        this.manipulatorState = manipulatorState;
        this.clawOpen = clawOpen;

        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        manipulator.setPosition(manipulatorState);
        manipulator.setClawOpen(clawOpen);
        manipulator.manipulatorTimer.startTime();
    }

    public boolean isFinished() {
        return true;
    }
}
