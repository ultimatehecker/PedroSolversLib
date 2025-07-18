package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.ElevatorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.solversHardware.SolversMotor;
import com.seattlesolvers.solverslib.solversHardware.SolversServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.constansts.ElevatorConstants;

public class Elevator extends SubsystemBase {
    public enum ElevatorState {
        RETRACTED,
        TRANSFER,
        SPECIMAN_LOW_READY,
        SPECIMAN_LOW_SCORE,
        SPECIMAN_HIGH_READY,
        SPECIMAN_HIGH_SCORE,
        SAMPLE_LOW_SCORE,
        SAMPLE_HIGH_SCORE
    }

    private SolversMotor elevatorMotor;
    private Motor.Encoder elevatorEncoder;
    private TouchSensor homingSensor;

    private SolversServo leftArmServo;
    private SolversServo rightArmServo;
    private SolversServo wristServo;
    private SolversServo clawServo;

    private final PIDFController elevatorController = new PIDFController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D, ElevatorConstants.F);

    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.105, 0.165, 0.0, 0.0);
    public ElevatorState elevatorState;

    private double target;
    private boolean elevatorReached;
    private boolean elevatorRetracted;

    public ElapsedTime elevatorTimer;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    public Elevator(HardwareMap aHardwareMap, TelemetryManager telemetryManager) {
        //elevatorMotor = new Motor(aHardwareMap, ElevatorConstants.elevatorMotorID, Motor.GoBILDA.RPM_312);
        elevatorMotor = new SolversMotor(aHardwareMap.get(DcMotor.class, ElevatorConstants.elevatorMotorID), 0.01);
        elevatorEncoder = new Motor(aHardwareMap, ElevatorConstants.elevatorMotorID).encoder;
        homingSensor = aHardwareMap.get(TouchSensor.class, ElevatorConstants.elevatorLimitSwitchID);

        leftArmServo = new SolversServo(aHardwareMap.get(Servo.class, "leftouttakeArm"), 0.01);
        rightArmServo = new SolversServo(aHardwareMap.get(Servo.class, "rightouttakeArm"), 0.01);
        wristServo = new SolversServo(aHardwareMap.get(Servo.class, "outtakeWrist"), 0.01);
        clawServo = new SolversServo(aHardwareMap.get(Servo.class, "outtakeClaw"), 0.01);

        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.REVERSE);

        elevatorController.setTolerance(25);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorEncoder.reset();

        this.telemetryManager = telemetryManager;
        elevatorTimer = new ElapsedTime();
        elevatorTimer.seconds();
    }

    public int getPosition() {
        return elevatorEncoder.getPosition();
    }

    public double getCorrectedVelocity() {
        return elevatorEncoder.getCorrectedVelocity();
    }

    public void setTargetPosition(ElevatorState elevatorState) {
        double unClippedTarget;
        switch (elevatorState) {
            case RETRACTED:
                unClippedTarget = ElevatorConstants.retractionHeight;
                break;
            case TRANSFER:
                unClippedTarget = ElevatorConstants.transferHeight;
                break;
            case SPECIMAN_LOW_READY:
                unClippedTarget = ElevatorConstants.lowSpecimanReady;
                break;
            case SPECIMAN_LOW_SCORE:
                unClippedTarget = ElevatorConstants.lowSpecimanScore;
                break;
            case SPECIMAN_HIGH_READY:
                unClippedTarget = ElevatorConstants.highSpecimanReady;
                break;
            case SPECIMAN_HIGH_SCORE:
                unClippedTarget = ElevatorConstants.highSpecimanScore;
                break;
            case SAMPLE_LOW_SCORE:
                unClippedTarget = ElevatorConstants.lowBucketHeight;
                break;
            case SAMPLE_HIGH_SCORE:
                unClippedTarget = ElevatorConstants.highBucketHeight;
                break;
            default:
                unClippedTarget = 0;
        }

        this.target = Range.clip(unClippedTarget, 0.0, 4000);
        this.elevatorState = elevatorState;
        this.elevatorReached = false;
    }

    public void toPosition() {
        double power = elevatorController.calculate(getPosition(), target);

        if (target == 0 && !isReached()) {
            power -= 0.1;
        }

        if (isRetracted() && isHomingSwitchTriggered()) {
            elevatorMotor.setPower(0);
            elevatorEncoder.reset();
        } else {
            elevatorMotor.setPower(power + elevatorFeedforward.calculate(getCorrectedVelocity()));
        }
    }

    public boolean isHomingSwitchTriggered() {
        return homingSensor.isPressed();
    }

    public boolean isReached() {
        /*
        return elevatorReached = elevatorController.atSetPoint() && (getCorrectedVelocity() < 1)
                || (target == 0 && getPosition() < 15 && isHomingSwitchTriggered())
                || (getPosition() >= target && (target == ElevatorConstants.highBucketHeight || target == ElevatorConstants.lowBucketHeight))
                || (target == ElevatorConstants.transferHeight + 50 && getPosition() > ElevatorConstants.transferHeight && getPosition() < ElevatorConstants.transferHeight + 65);
         */
         return elevatorReached = elevatorController.atSetPoint() && getCorrectedVelocity() == 0;
    }

    public boolean isRetracted() {
        return elevatorRetracted = (target <= 0) && elevatorReached && getCorrectedVelocity() == 0;
    }

    public void onInit() {
        setTargetPosition(ElevatorState.RETRACTED);
        elevatorEncoder.reset();
    }


    @Override
    public void periodic() {
        telemetryManager.graph("Elevator Position", getPosition());
        telemetryManager.graph("Elevator Target", target);
        telemetryManager.debug("Elevator Timer: " + elevatorTimer.time());
        telemetryManager.debug("PID is there: " + elevatorController.atSetPoint());
        telemetryManager.debug("Elevator Position: " + getPosition());
        telemetryManager.debug("Elevator Velocity: " + getCorrectedVelocity());
        telemetryManager.debug("Elevator Target: " + target);
        telemetryManager.debug("Elevator Reached: " + isReached());
        telemetryManager.debug("Elevator Retracted: " + isRetracted());
        telemetryManager.debug("Elevator Homing Sensor: " + isHomingSwitchTriggered());
        telemetryManager.debug("Elevator Power: " + elevatorMotor.getPower());
        telemetryManager.debug("Feedforward Output: " + elevatorFeedforward.calculate(getCorrectedVelocity()));
    }
}
