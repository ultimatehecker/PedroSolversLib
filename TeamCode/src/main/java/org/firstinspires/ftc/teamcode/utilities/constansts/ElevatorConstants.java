package org.firstinspires.ftc.teamcode.utilities.constansts;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;

@Configurable
public class ElevatorConstants {
    public static final String elevatorMotorID = "viperMotor";
    public static final String elevatorLimitSwitchID = "elevatorLimitSwitch";

    public static double retractionHeight = 0;
    public static double transferHeight = 400;
    public static double lowSpecimanReady = 30;
    public static double lowSpecimanScore = 900;
    public static double highSpecimanReady = 1100;
    public static double highSpecimanScore = 2290;
    public static double lowBucketHeight = 1000;
    public static double highBucketHeight = 3300;

    public static double P = 0.01;
    public static double I = 0.00;
    public static double D = 0.0004;
    public static double F = 0.0; // 0.0002;
}
