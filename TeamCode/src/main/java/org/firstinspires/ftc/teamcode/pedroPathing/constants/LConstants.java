package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

import org.firstinspires.ftc.teamcode.utilities.constansts.DrivetrainConstants;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0029;
        ThreeWheelConstants.strafeTicksToInches = 0.0029;
        ThreeWheelConstants.turnTicksToInches = 0.003;
        ThreeWheelConstants.leftY = 6.375;
        ThreeWheelConstants.rightY = -6.375;
        ThreeWheelConstants.strafeX = -6.0625;
        ThreeWheelConstants.leftEncoder_HardwareMapName = DrivetrainConstants.fLMotorID;
        ThreeWheelConstants.rightEncoder_HardwareMapName = DrivetrainConstants.bRMotorID;
        ThreeWheelConstants.strafeEncoder_HardwareMapName = DrivetrainConstants.fRMotorID;
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




