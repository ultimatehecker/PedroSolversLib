package org.firstinspires.ftc.teamcode;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveToVisionTarget;
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
import static org.firstinspires.ftc.teamcode.utilities.constansts.GlobalConstants.*;

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
    private GamepadButton autoIntaking;

    private TelemetryManager telemetryManager;
    private long lastTime = 0;

    @Override
    public void initialize() {
        opModeType = OpModeType.TELEOP;
        telemetryManager = Panels.getTelemetry();

        drivetrain = Drivetrain.getInstance(hardwareMap, telemetryManager);
        elevator = Elevator.getInstance(hardwareMap, telemetryManager);
        manipulator = Manipulator.getInstance(hardwareMap, telemetryManager);
        intake = Intake.getInstance(hardwareMap, telemetryManager);
        vision = Vision.getInstance(hardwareMap, telemetryManager);

        intake.onInit();
        elevator.onInit();
        manipulator.onInit();

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        prepareSpeciman = new GamepadButton(driverController, GamepadKeys.Button.LEFT_BUMPER);
        scoreSpeciman = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_BUMPER);
        toggleSpecimanSample = new GamepadButton(driverController, GamepadKeys.Button.DPAD_LEFT);
        prepareIntake = new GamepadButton(driverController, GamepadKeys.Button.X);
        pickSample = new GamepadButton(driverController, GamepadKeys.Button.A);
        retractAndTransfer = new GamepadButton(driverController, GamepadKeys.Button.Y);

        intakeClaw = new GamepadButton(operatorController, GamepadKeys.Button.CROSS);
        manuipulatorClaw = new GamepadButton(operatorController, GamepadKeys.Button.TRIANGLE);
        autoIntaking = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_STICK_BUTTON);

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

        autoIntaking.whenPressed(new DriveToVisionTarget(drivetrain, vision, telemetryManager));

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
    public void run() {
        CommandScheduler.getInstance().run();

        long currentTime = System.nanoTime();
        double loopTimeMs = (currentTime - lastTime) / 1_000_000.0;
        lastTime = currentTime;

        telemetryManager.debug("Loop Time (ms): " + loopTimeMs);
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
