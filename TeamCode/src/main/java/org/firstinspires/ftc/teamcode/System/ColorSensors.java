
package org.firstinspires.ftc.teamcode.System;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class ColorSensors {

    private final NormalizedColorSensor sensor1;
    private final NormalizedColorSensor sensor2;
    private final NormalizedColorSensor sensor3;
    private final NormalizedColorSensor sensor4;

    public ColorSensors(HardwareMap hardwareMap) {

        sensor1 = hardwareMap.get(
                NormalizedColorSensor.class, "Color Sensor 1");

        sensor2 = hardwareMap.get(
                NormalizedColorSensor.class, "Color Sensor 2");

        sensor3 = hardwareMap.get(
                NormalizedColorSensor.class, "Color Sensor 3");

        sensor4 = hardwareMap.get(
                NormalizedColorSensor.class, "Color Sensor 4");
    }

    private NormalizedColorSensor getSensor(int number) {
        switch (number) {
            case 1:
                return sensor1;
            case 2:
                return sensor2;
            case 3:
                return sensor3;
            case 4:
                return sensor4;
            default:
                throw new IllegalArgumentException(
                        "Invalid color sensor: " + number);
        }
    }

    public NormalizedRGBA getNormalizedColors(int i) {
        return getSensor(i).getNormalizedColors();
    }

    public double getHue(NormalizedRGBA normalizedColors) {
        return JavaUtil.colorToHue(normalizedColors.toColor());
    }

    public String getColor(double hue) {
        if(hue < 30){
            return("red");
        }
        else if (hue < 60) {
            return("orange");
        }
        else if (hue < 90){
            return("yellow");
        }
        else if (hue < 150){
            return("green");
        }
        else if (hue < 225){
            return("blue");
        }
        else if (hue < 350){
            return("purple");
        }
        else{
            return("red");
        }

    }
}
/*
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
*/