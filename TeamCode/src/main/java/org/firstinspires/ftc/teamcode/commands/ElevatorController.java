package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorController extends CommandBase {
    private final Elevator elevator;
    private final Elevator.ElevatorState elevatorState;

    private ElapsedTime elevatorTimer;

    public ElevatorController(Elevator elevator, Elevator.ElevatorState elevatorState) {
        this.elevator = elevator;
        this.elevatorState = elevatorState;
        this.elevatorTimer = new ElapsedTime();

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetPosition(elevatorState);
        elevatorTimer.startTime();
    }

    @Override
    public void execute() {
        elevator.toPosition();
    }

    @Override
    public boolean isFinished() {
        return elevator.isReached();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorTimer.reset();
    }
}
