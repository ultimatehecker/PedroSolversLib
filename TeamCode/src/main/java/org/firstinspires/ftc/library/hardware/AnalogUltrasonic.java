package org.firstinspires.ftc.library.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.Queue;

public class AnalogUltrasonic {
    private final AnalogInput device;

    private DistanceUnit unit = DistanceUnit.INCH;
    public int rollingAverageSize = 3;
    private double deviceAverage;

    private final Queue<Double> deviceData = new LinkedList<>();

    public AnalogUltrasonic(HardwareMap aHardwareMap, String name) {
        this.device = aHardwareMap.get(AnalogInput.class, name);
    }

    public void calculateAverage() {
        deviceData.add(device.getVoltage());
        if (deviceData.size() > rollingAverageSize) {
            deviceData.remove();
        }

        deviceAverage = deviceData.stream().reduce((total, el) -> total + el / deviceData.size()).get();
    }

    public void setDistanceUnit(DistanceUnit unit) {
        this.unit = unit;
    }

    public double getDeviceAverage() {
        return unit.fromCm(deviceAverage * 500 / 3.3);
    }
}