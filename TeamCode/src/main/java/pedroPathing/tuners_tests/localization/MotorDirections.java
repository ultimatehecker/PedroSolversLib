package pedroPathing.tuners_tests.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.MecanumConstants;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

import pedroPathing.Constants;

@TeleOp(name = "Motor Directions", group = "Teleop Test")
public class MotorDirections extends OpMode {
    MecanumConstants constants;

    private Telemetry telemetryA;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    @Override
    public void init() {
        constants = Constants.driveConstants;

        leftFront = hardwareMap.get(DcMotorEx.class, constants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, constants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, constants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, constants.rightFrontMotorName);
        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will allow you to test the directions of your motors. You can change the directions in FTCDashboard -> FollowerConstants.");
        telemetryA.update();
    }

    @Override
    public void loop() {
        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);

        if(gamepad1.a)
            leftFront.setPower(1);
        else
            leftFront.setPower(0);

        if(gamepad1.y)
            leftRear.setPower(1);
        else
            leftRear.setPower(0);

        if(gamepad1.b)
            rightFront.setPower(1);
        else
            rightFront.setPower(0);

        if(gamepad1.x)
            rightRear.setPower(1);
        else
            rightRear.setPower(0);

        telemetryA.addLine("Press A to spin the left front motor at 100% power");
        telemetryA.addLine("Press Y to spin the left rear motor at 100% power");
        telemetryA.addLine("Press B to spin the right front motor at 100% power");
        telemetryA.addLine("Press X to spin the right rear motor at 100% power");
        telemetryA.addData("Left Front Motor Direction: ", constants.leftFrontMotorDirection);
        telemetryA.addData("Left Rear Motor Direction: ", constants.leftRearMotorDirection);
        telemetryA.addData("Right Front Motor Direction: ", constants.rightFrontMotorDirection);
        telemetryA.addData("Right Rear Motor Direction: ", constants.rightRearMotorDirection);
        telemetryA.update();
    }
}
