package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.solversHardware.SolversMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.utilities.Units;
import org.firstinspires.ftc.teamcode.utilities.constansts.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.utilities.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utilities.geometry.Rotation2d;

import java.util.Collections;
import java.util.Set;

public class Drivetrain extends SubsystemBase {
    private SolversMotor leftFront;
    private SolversMotor rightFront;
    private SolversMotor leftRear;
    private SolversMotor rightRear;

    public Follower follower;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    private static Drivetrain instance = null;
    public static synchronized Drivetrain getInstance(HardwareMap aHardwareMap, TelemetryManager telemetryManager) {
        if(instance == null) {
            instance = new Drivetrain(aHardwareMap, telemetryManager);
        }

        return instance;
    }

    private Drivetrain(HardwareMap aHardwareMap, TelemetryManager telemetryManager) {
        leftFront = new SolversMotor(aHardwareMap.get(DcMotor.class, DrivetrainConstants.fLMotorID), 0.01);
        rightFront = new SolversMotor(aHardwareMap.get(DcMotor.class, DrivetrainConstants.fRMotorID), 0.01);
        leftRear = new SolversMotor(aHardwareMap.get(DcMotor.class, DrivetrainConstants.bLMotorID), 0.01);
        rightRear = new SolversMotor(aHardwareMap.get(DcMotor.class, DrivetrainConstants.bRMotorID), 0.01);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = new Follower(aHardwareMap, FConstants.class, LConstants.class);
        this.telemetryManager = telemetryManager;
    }

    @Override
    public void periodic() {
        telemetryManager.debug("Drivetrain Pose X: " + getPose().getX());
        telemetryManager.debug("Drivetrain Pose Y: " + getPose().getY());
        telemetryManager.debug("Drivetrain Pose θ: " + getPose().getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return new Pose2d(follower.getPose().getX(), follower.getPose().getY(), new Rotation2d(follower.getPose().getHeading()));
    }

    public void setStartingPose(Pose2d pose) {
        follower.setStartingPose(pose.getAsPedroPose());
    }

    public void setMovementVectors(double forward, double strafe, double rotation, boolean isRobotCentric) {
        follower.setTeleOpMovementVectors(forward, strafe, rotation, isRobotCentric);
    }

    public void setMovementVectors(double forward, double strafe, double rotation) {
        setMovementVectors(forward, strafe, rotation, true);
    }

    public void resetPose(Pose2d pose) {
        Pose pedroPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        follower.setPose(pedroPose);
    }

    public void onStart() {
        follower.startTeleopDrive();
        follower.update();
    }
}
