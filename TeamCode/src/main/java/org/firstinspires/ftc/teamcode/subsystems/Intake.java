package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.solversHardware.SolversServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.constansts.IntakeConstants;
import org.firstinspires.ftc.teamcode.utilities.constansts.ManipulatorConstants;

public class Intake extends SubsystemBase {
    public enum IntakeState {
        STORED,
        TRANSFER,
        HOVER_OUT,
        HOVER_IN,
        INTAKING_IN,
        INTAKING_OUT
    }

    public enum WristState {
        NORMAL,
        DEG30,
        DEG60,
        DEG90
    }

    private SolversServo leftLinkageServo;
    private SolversServo rightLinkageServo;
    private SolversServo leftArmServo;
    private SolversServo rightArmServo;
    private SolversServo wristServo;
    private SolversServo clawServo;

    private Telemetry telemetry;
    public Timer intakeTimer;
    private boolean isClawOpen;

    private IntakeState intakeState;
    private WristState wristState;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    public Intake(HardwareMap aHardwareMap, Telemetry telemetry) {
        leftLinkageServo = new SolversServo(aHardwareMap.get(Servo.class, "linkageServo"), 0.01);
        rightLinkageServo = new SolversServo(aHardwareMap.get(Servo.class, "linkageServo2"), 0.01);
        leftArmServo = new SolversServo(aHardwareMap.get(Servo.class, "leftPivotingServo"), 0.01);
        rightArmServo = new SolversServo(aHardwareMap.get(Servo.class, "rightPivotingServo"), 0.01);
        wristServo = new SolversServo(aHardwareMap.get(Servo.class, "intakeArm"), 0.01);
        clawServo = new SolversServo(aHardwareMap.get(Servo.class, "intakeClaw"), 0.01);

        leftLinkageServo.setDirection(Servo.Direction.FORWARD);
        rightLinkageServo.setDirection(Servo.Direction.REVERSE);
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        intakeState = IntakeState.STORED;
        wristState = WristState.NORMAL;
        isClawOpen = true;

        intakeTimer = new Timer();
        this.telemetry = telemetry;
        telemetryManager = Panels.getTelemetry();
    }

    public void setIntakeState(IntakeState intakeState) {
        switch(intakeState) {
            case STORED:
                leftLinkageServo.setPosition(IntakeConstants.intakeLinkageServoInPosition);
                rightLinkageServo.setPosition(IntakeConstants.intakeLinkageServoInPosition);
                leftArmServo.setPosition(IntakeConstants.intakeArmStorePosition);
                rightArmServo.setPosition(IntakeConstants.intakeArmStorePosition);
            case TRANSFER:
                leftLinkageServo.setPosition(IntakeConstants.intakeLinkageServoInPosition);
                rightLinkageServo.setPosition(IntakeConstants.intakeLinkageServoInPosition);
                leftArmServo.setPosition(IntakeConstants.intakeArmTransferPosition);
                rightArmServo.setPosition(IntakeConstants.intakeArmTransferPosition);
            case HOVER_OUT:
                leftLinkageServo.setPosition(IntakeConstants.intakeLinkageServoOutPosition);
                rightLinkageServo.setPosition(IntakeConstants.intakeLinkageServoOutPosition);
                leftArmServo.setPosition(IntakeConstants.intakeArmReadyPosition);
                rightArmServo.setPosition(IntakeConstants.intakeArmReadyPosition);
            case HOVER_IN:
                leftLinkageServo.setPosition(IntakeConstants.intakeLinkageServoInPosition);
                rightLinkageServo.setPosition(IntakeConstants.intakeLinkageServoInPosition);
                leftArmServo.setPosition(IntakeConstants.intakeArmReadyPosition);
                rightArmServo.setPosition(IntakeConstants.intakeArmReadyPosition);
            case INTAKING_IN:
                leftLinkageServo.setPosition(IntakeConstants.intakeLinkageServoInPosition);
                rightLinkageServo.setPosition(IntakeConstants.intakeLinkageServoInPosition);
                leftArmServo.setPosition(IntakeConstants.intakeArmIntakingPosition);
                rightArmServo.setPosition(IntakeConstants.intakeArmIntakingPosition);
            case INTAKING_OUT:
                leftLinkageServo.setPosition(IntakeConstants.intakeLinkageServoOutPosition);
                rightLinkageServo.setPosition(IntakeConstants.intakeLinkageServoOutPosition);
                leftArmServo.setPosition(IntakeConstants.intakeArmIntakingPosition);
                rightArmServo.setPosition(IntakeConstants.intakeArmIntakingPosition);
        }

        this.intakeState = intakeState;
    }

    public void setWristState(WristState wristState) {
        switch(wristState) {
            case NORMAL:
                wristServo.setPosition(IntakeConstants.intakeWristRegularPosition);
            case DEG30:
                wristServo.setPosition(IntakeConstants.intakeWristAngled30Position);
            case DEG60:
                wristServo.setPosition(IntakeConstants.intakeWristAngled60Position);
            case DEG90:
                wristServo.setPosition(IntakeConstants.intakeWristAngled90Position);
        }

        this.wristState = wristState;
    }

    public void setClawOpen(boolean open) {
        if (open) {
            clawServo.setPosition(IntakeConstants.intakeClawOpenedPosition);
        } else {
            clawServo.setPosition(IntakeConstants.intakeClawClosedPosition);
        }

        this.isClawOpen = open;
    }

    public boolean isClawOpen() {
        return isClawOpen;
    }

    public void onInit() {
        setIntakeState(IntakeState.STORED);
        setWristState(WristState.NORMAL);
        setClawOpen(true);
    }

    @Override
    public void periodic() {

    }
}
