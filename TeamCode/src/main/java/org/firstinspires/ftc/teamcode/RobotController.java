package org.firstinspires.ftc.teamcode;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.DrivetrainController;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@TeleOp(name="RobotController")
public class RobotController extends CommandOpMode {
    private Drivetrain drivetrain;
    private Elevator elevator;

    private GamepadEx driverController;
    private GamepadEx operatorController;

    private TelemetryManager telemetryManager;

    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        telemetryManager = Panels.getTelemetry();

        register(drivetrain);
        schedule(new RunCommand(() -> telemetryManager.update(telemetry)));

        drivetrain.setDefaultCommand(new DrivetrainController(
                drivetrain,
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX
        ));
    }

    @Override
    public void runOpMode() {
        initialize();

        while (!opModeIsActive()) {
            drivetrain.drawCurrentTrajectory();
        }

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }

        reset();
    }
}
