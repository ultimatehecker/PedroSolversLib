package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerBuilder;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.drivetrain.MecanumConstants;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .centripetalScaling(0.005)
            .forwardZeroPowerAcceleration(-166.4825434)
            .lateralZeroPowerAcceleration(-164.2874314)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFSwitch(2)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.0, 0))
            .translationalPIDFFeedForward(0.015)
            .useSecondaryHeadingPIDF(true)
            .headingPIDFSwitch(2)
            .headingPIDFCoefficients(new PIDFCoefficients(2,0,0.1,0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0.0, 0.0, 0.0))
            .headingPIDFFeedForward(0.015)
            .useSecondaryDrivePIDF(true)
            .drivePIDFSwitch(2)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0,0.6,0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0.0, 0.0, 0.0, 0))
            .drivePIDFFeedForward(0.015)
            .mass(10.788597533);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xMovement(60.3993)
            .yMovement(44.24685)
            .maxPower(1)
            .leftFrontMotorName("leftFront")
            .rightFrontMotorName("rightFront")
            .leftRearMotorName("leftRear")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .motorCachingThreshold(0.01)
            .useBrakeModeInTeleOp(false);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardY(-6.375)
            .strafeX(-6.0625)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 3, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}