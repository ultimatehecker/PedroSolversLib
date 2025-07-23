package org.firstinspires.ftc.teamcode.utilities.constansts;

public class GlobalConstants {
    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public enum AllianceColor {
        RED,
        BLUE
    }

    public static OpModeType opModeType;
    public static boolean specimenTeleop = false;
    public static AllianceColor allianceColor;
}
