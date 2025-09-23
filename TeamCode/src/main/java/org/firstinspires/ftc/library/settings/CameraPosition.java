package org.firstinspires.ftc.library.settings;

public class CameraPosition {
    private final double x;
    private final double y;
    private final double z;
    private final double yaw;
    private final double pitch;
    private final double roll;

    public CameraPosition(final double x, final double y, final double z, final double yaw, final double pitch, final double roll) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public double getRoll() {
        return roll;
    }
}
