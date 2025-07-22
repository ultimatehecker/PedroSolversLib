package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.autonomous.paths.Trajectories;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.function.BooleanSupplier;

@Autonomous(name="Autonomous", group="Pedro Pathing")
public class SelectableAutonomous extends CommandOpMode {
    private Drivetrain drivetrain;
    private Elevator elevator;
    private Manipulator manipulator;
    private Intake intake;
    private Vision vision;

    private boolean autoChoosen;
    private boolean leftBumperPrevious;
    private boolean rightBumperPrevious;
    private boolean aButtonPrevious;

    private PathChain pathChain;
    private AutonomousMode currentAuto;
    private static AutonomousMode[] AUTONOMOUS_MODES;
    private int autoIndex = 0;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @Override
    public void initialize() {
        telemetryManager = Panels.getTelemetry();
        drivetrain = Drivetrain.getInstance(hardwareMap, telemetryManager);
        elevator = Elevator.getInstance(hardwareMap, telemetryManager);
        manipulator = Manipulator.getInstance(hardwareMap, telemetryManager);
        intake = Intake.getInstance(hardwareMap, telemetryManager);
        vision = Vision.getInstance(hardwareMap, telemetryManager);

        autoChoosen = false;
        pathChain = Trajectories.fourBucketNoPark;
        currentAuto = AutonomousMode.FOUR_BUCKET_NO_PARK;
        AUTONOMOUS_MODES = AutonomousMode.values();

        intake.onInit();
        elevator.onInit();
        manipulator.onInit();
    }

    @Override
    public void initialize_loop() {
        if(!autoChoosen) {
            if(gamepad2.right_bumper && !rightBumperPrevious) {
                nextAuto();
            } else if(gamepad2.left_bumper && !leftBumperPrevious) {
                previousAuto();
            } else if(gamepad2.a && !aButtonPrevious) {
                schedule(
                        new RunCommand(drivetrain.follower::update),
                        currentAuto.getCommand(drivetrain, elevator, intake, (BooleanSupplier) this::opModeIsActive)
                );

                drivetrain.setStartingPose(currentAuto.getStartingPose());
                autoChoosen = true;
            }

            rightBumperPrevious = gamepad2.right_bumper;
            leftBumperPrevious = gamepad2.left_bumper;
            aButtonPrevious = gamepad2.a;

            telemetryManager.debug("Current Autonomous: ", currentAuto.toString());
            telemetryManager.debug("\n");
            telemetryManager.debug("When the selection is made, press the A button to confirm. Auto is choosen: " + autoChoosen);
            telemetryManager.update(telemetry);
        } else {
            telemetryManager.getCanvas().clear();
            telemetryManager.debug("Selected Auto [" + autoIndex + "/" + (AUTONOMOUS_MODES.length - 1) + "]: " + currentAuto.toString());
            telemetryManager.debug("\n");
            telemetryManager.debug("Starting Pose X: " + drivetrain.getPose().getX());
            telemetryManager.debug("Starting Pose Y: " + drivetrain.getPose().getY());
            telemetryManager.debug("Starting Pose θ: " + drivetrain.getPose().getRotation().getDegrees());
            telemetryManager.update(telemetry);

            drivetrain.drawCurrentTrajectory();
        }

        drivetrain.follower.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // run the scheduler
        try {
            while (opModeInInit()) {
                initialize_loop();
            }
            while (!isStopRequested() && opModeIsActive()) {
                run();
            }

        } finally {
            try {
                end();
            } finally {
                reset();
            }
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryManager.update(telemetry);
    }

    public void nextAuto() {
        autoIndex = (autoIndex + 1) % AUTONOMOUS_MODES.length;
        currentAuto = AUTONOMOUS_MODES[autoIndex];
    }

    public void previousAuto() {
        autoIndex = (autoIndex - 1 + AUTONOMOUS_MODES.length) % AUTONOMOUS_MODES.length;
        currentAuto = AUTONOMOUS_MODES[autoIndex];
    }
}
