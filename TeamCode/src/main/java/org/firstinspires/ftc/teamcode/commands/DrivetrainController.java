package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DrivetrainController extends CommandBase {
    private Drivetrain drivetrain;
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;

    public DrivetrainController(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.onStart();
    }

    @Override
    public void execute() {
        drivetrain.setMovementVectors(forward.getAsDouble(), -strafe.getAsDouble(), -rotation.getAsDouble() * 0.6, false);
        drivetrain.follower.update();
    }
}
