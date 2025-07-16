package org.firstinspires.ftc.teamcode.utilities.tuning.pidf;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.solversHardware.SolversMotor;

import org.firstinspires.ftc.teamcode.utilities.constansts.ElevatorConstants;

//@Disabled
@TeleOp(name="ElevatorFF-Tuning")
public class ElevatorFeedforward extends OpMode {
    private SolversMotor viperMotor;
    private Motor.Encoder viperEncoder;

    private double power;
    private double saveSlot1, saveSlot2 = -1;

    private boolean aButtonPreviousState = false;
    private boolean xButtonPreviousState, yButtonPreviousState = false;
    private boolean leftBumperButtonPreviousState, rightBumperButtonPreviousState = false;
    private boolean isFinishedTuning = false;
    private boolean startButtonPressed = false;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    @Override
    public void init() {
        viperMotor = new SolversMotor(hardwareMap.get(DcMotor.class, ElevatorConstants.elevatorMotorID));
        viperEncoder = new Motor(hardwareMap, ElevatorConstants.elevatorMotorID).encoder;

        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperEncoder.reset();

        telemetryManager = Panels.getTelemetry();
        power = 0.0;
    }

    @Override
    public void init_loop() {
        if(startButtonPressed) {
            telemetryManager.getCanvas().clear();
            telemetryManager.debug("Square (X): Increase power by 0.01");
            telemetryManager.debug("Triangle (Y): Decrease power by 0.01");
            telemetryManager.debug("Cross (A): Continue / output final results (need both save slots full)");
            telemetryManager.debug("Left Bumper: Save slot 1 (-1 represents no data saved)");
            telemetryManager.debug("Right Bumper: Save slot 2 (-1 represents no data saved)");
            telemetryManager.update(telemetry);
        } else {
            telemetryManager.debug("This will run the motor specified initially at 0 power.");
            telemetryManager.debug("Make sure the motor is able to rotate multiple times without damaging the system.");
            telemetryManager.debug("Increase power until the elevator barely moves up, save it, then decrease power until it barely moves down, and save it.");
            telemetryManager.debug("Press the start button to bring up the controls");
            telemetryManager.update(telemetry);

            if(gamepad1.start) {
                startButtonPressed = true;
            }
        }
    }

    @Override
    public void loop() {
        if (gamepad1.square && !xButtonPreviousState) {
            if (power == 1) {
                power -= 0;
            } else {
                power += 0.01;
            }
        } else if (gamepad1.triangle && !yButtonPreviousState) {
            if (power == 0) {
                power -= 0;
            } else {
                power -= 0.01;
            }
        }

        if(gamepad1.left_bumper && !leftBumperButtonPreviousState) {
            saveSlot1 = power;
        } else if(gamepad1.right_bumper && !rightBumperButtonPreviousState) {
            saveSlot2 = power;
        }

        if(gamepad1.a && !aButtonPreviousState || isFinishedTuning) {
            if(saveSlot1 != -1 && saveSlot2 != -1) {
                double min = Math.min(saveSlot1, saveSlot2);
                double max = Math.max(saveSlot1, saveSlot2);

                double kG = (max + min) / 2;
                double kS = (max - min) / 2;

                telemetryManager.getCanvas().clear();
                telemetryManager.debug("Calculated kG Value: " + kG);
                telemetryManager.debug("Calculated kS Value: " + kS);
                telemetryManager.update(telemetry);

                isFinishedTuning = true;
            }
        } else {
            viperMotor.setPower(power);

            telemetryManager.getCanvas().clear();
            telemetryManager.debug("Elevator Motor Power: " + power);
            telemetryManager.debug("Elevator Current Position: " + viperEncoder.getPosition());
            telemetryManager.debug("Elevator Current Velocity: " + viperEncoder.getCorrectedVelocity());
            telemetryManager.debug("\n");
            telemetryManager.debug("Save Slot 1: " + saveSlot1);
            telemetryManager.debug("Save Slot 2: " + saveSlot2);
            telemetryManager.update(telemetry);
        }

        aButtonPreviousState = gamepad1.a;
        xButtonPreviousState = gamepad1.square;
        yButtonPreviousState = gamepad1.triangle;
        leftBumperButtonPreviousState = gamepad1.left_bumper;
        rightBumperButtonPreviousState = gamepad1.right_bumper;
    }
}