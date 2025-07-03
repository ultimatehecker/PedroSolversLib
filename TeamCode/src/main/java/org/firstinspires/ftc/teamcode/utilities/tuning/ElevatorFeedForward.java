package org.firstinspires.ftc.teamcode.utilities.tuning;

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
public class ElevatorFeedForward extends OpMode {
    private SolversMotor viperMotor;
    private Motor.Encoder viperEncoder;
    private double power;

    private boolean aButtonPreviousState, bButtonPreviousState = false;
    private boolean xButtonPreviousState, yButtonPreviousState = false;

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
        telemetryManager.debug("This will run the motor specified initally at " + power + " power. Make sure the motor is able to rotate multiple times without damaging the system");
        telemetryManager.debug("Press square to increase power by 0.01, and triangle to decrease by 0.01");
        telemetryManager.debug("Press circle to increase power by 0.1, and cross to decrease by 0.1");
        telemetryManager.update(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.square && !aButtonPreviousState) {
            power += 0.01;
        } else if (gamepad1.triangle && !bButtonPreviousState) {
            power -= 0.01;
        }

        if (gamepad1.circle && !xButtonPreviousState) {
            power += 0.1;
        } else if (gamepad1.cross && !yButtonPreviousState) {
            power -= 0.1;
        }

        aButtonPreviousState = gamepad1.square;
        bButtonPreviousState = gamepad1.triangle;
        xButtonPreviousState = gamepad1.circle;
        yButtonPreviousState = gamepad1.cross;

        viperMotor.setPower(power);

        telemetryManager.debug("Power: " + power);
        telemetryManager.update(telemetry);
    }
}