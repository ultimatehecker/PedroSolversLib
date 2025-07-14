package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.IgnoreConfigurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.util.DashboardPoseTracker;

import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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

    private IMU imu;
    private BNO055IMU oldIMU;
    private boolean usingNewIMU;
    public Follower follower;

    private Telemetry telemetry;

    @IgnoreConfigurable
    static DashboardPoseTracker dashboardPoseTracker;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;

    public Drivetrain(HardwareMap aHardwareMap, Telemetry telemetry) {
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

        follower = Constants.createFollower(aHardwareMap);
        dashboardPoseTracker = follower.getDashboardPoseTracker();
        telemetryManager = Panels.getTelemetry();
        this.telemetry = telemetry;

        initializeIMU(aHardwareMap, true);
    }

    public void initializeIMU(HardwareMap aHardwareMap, boolean isNew) {
        if(isNew) {
            imu = aHardwareMap.get(IMU.class, DrivetrainConstants.imuID);

            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            ));

            usingNewIMU = true;
            imu.initialize(parameters);
        } else {
            oldIMU = aHardwareMap.get(BNO055IMU.class, DrivetrainConstants.imuID);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";

            usingNewIMU = false;
            oldIMU.initialize(parameters);
        }
    }

    @Override
    public void periodic() {
        drawCurrentAndHistoricalTrajectory();
    }

    public double getYawDegrees(AngleUnit angleUnit) {
        if(usingNewIMU()) {
            return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
        } else {
            return oldIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, angleUnit).firstAngle;
        }
    }
    public double getPitchDegrees(AngleUnit angleUnit) {
        if(usingNewIMU()) {
            return imu.getRobotYawPitchRollAngles().getPitch(angleUnit);
        } else {
            return oldIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, angleUnit).secondAngle;
        }
    }
    public double getRollDegrees(AngleUnit angleUnit) {
        if(usingNewIMU()) {
            return imu.getRobotYawPitchRollAngles().getRoll(angleUnit);
        } else {
            return oldIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, angleUnit).thirdAngle;
        }
    }

    public Pose2d getPose() {
        return new Pose2d(follower.getPose().getX(), follower.getPose().getY(), Rotation2d.fromDegrees(follower.getPose().getHeading()));
    }

    public void setMovementVectors(double forward, double strafe, double rotation, boolean isRobotCentric) {
        follower.setTeleOpDrive(forward, strafe, rotation, isRobotCentric);
    }

    public void setMovementVectors(double forward, double strafe, double rotation) {
        follower.setTeleOpDrive(forward, strafe, rotation, true);
    }

    public void resetPose(Pose2d pose) {
        Pose pedroPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        follower.setPose(pedroPose);
    }

    public void resetIMU() {
        if(usingNewIMU()) {
            imu.resetYaw();
        }
    }

    /* Draw on init_loop if planning to use */
    public void drawCurrentTrajectory() {
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
    }

    public void drawCurrentAndHistoricalTrajectory() {
        Drawing.drawPoseHistory(dashboardPoseTracker);
        drawCurrentTrajectory();
    }

    public boolean usingNewIMU() {
        return usingNewIMU;
    }

    public void onStart() {
        follower.startTeleopDrive();
        follower.update();
    }
}
