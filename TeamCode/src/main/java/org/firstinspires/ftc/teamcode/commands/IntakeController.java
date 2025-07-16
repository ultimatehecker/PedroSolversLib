package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeController extends CommandBase {
    private final Intake intake;
    private Intake.IntakeState intakeState;
    private Intake.WristState wristState;
    private boolean clawOpen;

    public IntakeController(Intake intake, Intake.IntakeState intakeState, Intake.WristState wristState, boolean clawOpen) {
        this.intake = intake;
        this.intakeState = intakeState;
        this.wristState = wristState;
        this.clawOpen = clawOpen;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeTimer.resetTimer();
    }

    @Override
    public void execute() {
        intake.setIntakeState(intakeState);
        intake.setWristState(wristState);
        intake.setClawOpen(clawOpen);
    }

    public boolean isFinished() {
        return true;
    }
}
