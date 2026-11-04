
package org.firstinspires.ftc.teamcode.System;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensors {

    private final ColorSensor sensor1;
    private final ColorSensor sensor2;
    private final ColorSensor sensor3;
    private final ColorSensor sensor4;

    public ColorSensors(HardwareMap hardwareMap) {

        sensor1 = hardwareMap.get(
                ColorSensor.class, "Color Sensor 1");

        sensor2 = hardwareMap.get(
                ColorSensor.class, "Color Sensor 2");

        sensor3 = hardwareMap.get(
                ColorSensor.class, "Color Sensor 3");

        sensor4 = hardwareMap.get(
                ColorSensor.class, "Color Sensor 4");
    }

    private ColorSensor getSensor(int number) {
        switch (number) {
            case 1: return sensor1;
            case 2: return sensor2;
            case 3: return sensor3;
            case 4: return sensor4;
            default:
                throw new IllegalArgumentException(
                        "Invalid color sensor: " + number);
        }
    }

    public int getRed(int number) {
        return getSensor(number).red();
    }

    public int getGreen(int number) {
        return getSensor(number).green();
    }

    public int getBlue(int number) {
        return getSensor(number).blue();
    }

    public int getAlpha(int number) {
        return getSensor(number).alpha();
    }
}
