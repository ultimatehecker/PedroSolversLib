package org.firstinspires.ftc.teamcode;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DrivetrainController;
import org.firstinspires.ftc.teamcode.commands.group.PickSample;
import org.firstinspires.ftc.teamcode.commands.group.PrepareIntake;
import org.firstinspires.ftc.teamcode.commands.group.PrepareSample;
import org.firstinspires.ftc.teamcode.commands.group.PrepareSpeciman;
import org.firstinspires.ftc.teamcode.commands.group.RetractIntakeAndTransfer;
import org.firstinspires.ftc.teamcode.commands.group.ScoreSampleThenRetract;
import org.firstinspires.ftc.teamcode.commands.group.ScoreSpecimanThenRetract;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name="RobotController")
public class RobotController extends CommandOpMode {
    private Drivetrain drivetrain;
    private Elevator elevator;
    private Manipulator manipulator;
    private Intake intake;
    private Vision vision;

    private Manipulator.SampleSpecimanState sampleSpecimanState = Manipulator.SampleSpecimanState.SPEC;

    private GamepadEx driverController;
    private GamepadEx operatorController;

    private GamepadButton prepareSpeciman;
    private GamepadButton scoreSpeciman;
    private GamepadButton toggleSpecimanSample;
    private GamepadButton prepareIntake;
    private GamepadButton pickSample;
    private GamepadButton retractAndTransfer;
    private GamepadButton intakeClaw;
    private GamepadButton manuipulatorClaw;

    private TelemetryManager telemetryManager;

    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        manipulator = new Manipulator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        vision = new Vision(hardwareMap, telemetry);

        intake.onInit();
        elevator.onInit();
        manipulator.onInit();

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        prepareSpeciman = new GamepadButton(driverController, GamepadKeys.Button.LEFT_BUMPER);
        scoreSpeciman = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_BUMPER);
        toggleSpecimanSample = new GamepadButton(driverController, GamepadKeys.Button.DPAD_LEFT);
        prepareIntake = new GamepadButton(driverController, GamepadKeys.Button.SQUARE);
        pickSample = new GamepadButton(driverController, GamepadKeys.Button.A);
        retractAndTransfer = new GamepadButton(driverController, GamepadKeys.Button.TRIANGLE);

        intakeClaw = new GamepadButton(operatorController, GamepadKeys.Button.CROSS);
        manuipulatorClaw = new GamepadButton(operatorController, GamepadKeys.Button.TRIANGLE);

        telemetryManager = Panels.getTelemetry();

        register(drivetrain, elevator, manipulator, intake, vision);

        toggleSpecimanSample.toggleWhenActive(
                new InstantCommand(() -> sampleSpecimanState = Manipulator.SampleSpecimanState.SAMP),
                new InstantCommand(() -> sampleSpecimanState = Manipulator.SampleSpecimanState.SPEC)
        );

        prepareSpeciman.whenPressed(new ConditionalCommand(
               new PrepareSpeciman(manipulator, elevator),
               new PrepareSample(manipulator, elevator),
               () -> sampleSpecimanState == Manipulator.SampleSpecimanState.SPEC
        ));


        scoreSpeciman.whenPressed(new ConditionalCommand(
                new ScoreSpecimanThenRetract(manipulator, elevator),
                new ScoreSampleThenRetract(manipulator, elevator),
                () -> sampleSpecimanState == Manipulator.SampleSpecimanState.SPEC
        ));

        prepareIntake.whenPressed(new PrepareIntake(intake, elevator));
        pickSample.whenPressed(new PickSample(intake));
        retractAndTransfer.whenPressed(new RetractIntakeAndTransfer(elevator, intake, manipulator));

        drivetrain.setDefaultCommand(new DrivetrainController(
                drivetrain,
                vision,
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX
        ));
    }

    @Override
    public void initialize_loop() {
        drivetrain.drawCurrentTrajectory();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryManager.debug("Game Piece: " + sampleSpecimanState.toString());
        telemetryManager.update(telemetry);

        CommandScheduler.getInstance().onCommandInitialize(command -> {
            telemetryManager.debug("Command Initialized: " + command.getName());
        });

        CommandScheduler.getInstance().onCommandFinish(command -> {
            telemetryManager.debug("Command Finished: " + command.getName());
        });
    }
}
