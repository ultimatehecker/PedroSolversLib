package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorController extends CommandBase {
    private final Elevator elevator;
    private final Elevator.ElevatorState elevatorState;

    private boolean tryingToFinish;

    public ElevatorController(Elevator elevator, Elevator.ElevatorState elevatorState) {
        this.elevator = elevator;
        this.elevatorState = elevatorState;
        this.tryingToFinish = false;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetPosition(elevatorState);
    }

    @Override
    public void execute() {
        elevator.toPosition();

        if(elevator.isReached()) {
            tryingToFinish = true;
            elevator.elevatorTimer.resetTimer();
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.elevatorTimer.getElapsedTimeSeconds() > 0.1 && tryingToFinish;
    }
}
