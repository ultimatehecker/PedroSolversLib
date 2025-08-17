package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.List;
import java.util.function.DoubleSupplier;

public class DrivetrainController extends CommandBase {
    private Drivetrain drivetrain;
    private Vision vision;
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;

    public DrivetrainController(Drivetrain drivetrain, Vision vision, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        drivetrain.onStart();
    }

    @Override
    public void execute() {
        drivetrain.setMovementVectors(forward.getAsDouble(), -strafe.getAsDouble(), -rotation.getAsDouble() * 0.6, false);
        List<Vision.Detection> found = vision.getDetectionsField(drivetrain.getPose());
        drivetrain.follower.update();
    }
}
