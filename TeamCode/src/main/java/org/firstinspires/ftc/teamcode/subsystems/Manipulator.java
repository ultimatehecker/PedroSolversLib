package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.solversHardware.SolversServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.constansts.ElevatorConstants;
import org.firstinspires.ftc.teamcode.utilities.constansts.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
    public enum ManipulatorState {
        TRANSFER,
        TRANSFER_CLEARENCE,
        SPECIMAN_READY,
        SPECIMAN_SCORE,
        SAMPLE_SCORE,
        WALL_INTAKE
    }

    public enum SampleSpecimanState {
        SAMP,
        SPEC
    }

    public ManipulatorState manipulatorState;

    private SolversServo leftArmServo;
    private SolversServo rightArmServo;
    private SolversServo wristServo;
    private SolversServo clawServo;

    private RevColorSensorV3 colorSensor;
    private AnalogInput absoluteEncoder;
    private TouchSensor elevatorResetSwitch;

    private boolean isClawOpen;
    private boolean hasSpeciman;

    public ElapsedTime manipulatorTimer;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private static Manipulator instance = null;
    public static synchronized Manipulator getInstance(HardwareMap aHardwareMap, TelemetryManager telemetryManager) {
        if(instance == null) {
            instance = new Manipulator(aHardwareMap, telemetryManager);
        }

        return instance;
    }

    private Manipulator(HardwareMap aHardwareMap, TelemetryManager telemetryManager) {
        leftArmServo = new SolversServo(aHardwareMap.get(Servo.class, "leftouttakeArm"), 0.01);
        rightArmServo = new SolversServo(aHardwareMap.get(Servo.class, "rightouttakeArm"), 0.01);
        wristServo = new SolversServo(aHardwareMap.get(Servo.class, "outtakeWrist"), 0.01);
        clawServo = new SolversServo(aHardwareMap.get(Servo.class, "outtakeClaw"), 0.01);

        colorSensor = (RevColorSensorV3) aHardwareMap.colorSensor.get("outtakeColorSensor");
        absoluteEncoder = aHardwareMap.get(AnalogInput.class, "absoluteEncoder");
        elevatorResetSwitch = aHardwareMap.get(TouchSensor.class, "elevatorResetSwitch");

        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.REVERSE);

        manipulatorState = ManipulatorState.TRANSFER;

        this.telemetryManager = telemetryManager;
        manipulatorTimer = new ElapsedTime();
    }

    public ManipulatorState getState() {
        return manipulatorState;
    }

    public void setPosition(ManipulatorState manipulatorState) {
        switch (manipulatorState) {
            case TRANSFER:
                leftArmServo.setPosition(ManipulatorConstants.manipulatorArmTransferPosition);
                rightArmServo.setPosition(ManipulatorConstants.manipulatorArmTransferPosition);
                wristServo.setPosition(ManipulatorConstants.manipulatorWristTransferPosition);
                break;
            case TRANSFER_CLEARENCE:
                leftArmServo.setPosition(ManipulatorConstants.manipulatorArmTransferPosition);
                rightArmServo.setPosition(ManipulatorConstants.manipulatorArmTransferPosition);
                wristServo.setPosition(ManipulatorConstants.manipulatorWristSpecimanPosition);
                break;
            case SPECIMAN_READY:
                leftArmServo.setPosition(ManipulatorConstants.manipulatorArmSpecimanPosition);
                rightArmServo.setPosition(ManipulatorConstants.manipulatorArmSpecimanPosition);
                wristServo.setPosition(ManipulatorConstants.manipulatorWristSpecimanPosition);
                break;
            case SPECIMAN_SCORE:
                leftArmServo.setPosition(ManipulatorConstants.manipulatorArmSpecimanPosition);
                rightArmServo.setPosition(ManipulatorConstants.manipulatorArmSpecimanPosition);
                wristServo.setPosition(ManipulatorConstants.manipulatorWristSpecimanPosition);
                break;
            case SAMPLE_SCORE:
                leftArmServo.setPosition(ManipulatorConstants.manipulatorArmBucketPosition);
                rightArmServo.setPosition(ManipulatorConstants.manipulatorArmBucketPosition);
                wristServo.setPosition(ManipulatorConstants.manipulatorWristBucketPosition);
                break;
            case WALL_INTAKE:
                leftArmServo.setPosition(ManipulatorConstants.manipulatorArmIntakingPosition);
                rightArmServo.setPosition(ManipulatorConstants.manipulatorArmIntakingPosition);
                wristServo.setPosition(ManipulatorConstants.manipulatorWristIntakingPosition);
                break;
        }

        this.manipulatorState = manipulatorState;
    }

    public void setClawOpen(boolean open) {
        if (open) {
            clawServo.setPosition(ManipulatorConstants.manipulatorClawOpenPosition);
        } else {
            clawServo.setPosition(ManipulatorConstants.manipulatorClawClosedPosition);
        }

        this.isClawOpen = open;
    }

    public boolean isClawOpen() {
        return isClawOpen;
    }

    /* TODO: Work in Progress (not currently on the robot) */
    public boolean clawHasSpeciman() {
        double redValue = colorSensor.red();
        double greenValue = colorSensor.green();
        double blueValue = colorSensor.blue();

        return true;
    }

    public void onInit() {
        setPosition(ManipulatorState.TRANSFER);
        setClawOpen(true);
    }

    @Override
    public void periodic() {
        telemetryManager.debug("Manipulator State: " + manipulatorState);
        telemetryManager.debug("Manipulator Claw Open: " + isClawOpen());
    }
}
