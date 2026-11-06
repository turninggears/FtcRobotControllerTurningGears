
package org.firstinspires.ftc.teamcode.System;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class ColorSensors {

    private final NormalizedColorSensor[] sensors =
            new NormalizedColorSensor[4];

    private final double BALL_ALPHA_THRESHOLD;

    public ColorSensors(HardwareMap hardwareMap, double BALL_ALPHA_THRESHOLD) {

        this.BALL_ALPHA_THRESHOLD = BALL_ALPHA_THRESHOLD;

        for (int i = 0; i < 4; i++) {

            try {
                String[] names = {
                        "Color Sensor 0",
                        "Color Sensor 1",
                        "Color Sensor 2",
                        "Color Sensor 3"
                };
                sensors[i] = hardwareMap.get(
                        NormalizedColorSensor.class,
                        names[i]
                );
            } catch (Exception e) {

                sensors[i] = null;
            }
        }
    }
    private NormalizedColorSensor getSensor(int number) {
        return sensors[number];
        }
    public NormalizedRGBA getNormalizedColors(int sensor) {
        return getSensor(sensor).getNormalizedColors();
    }

    public double calculateHue(NormalizedRGBA normalizedColors) {
        return JavaUtil.colorToHue(normalizedColors.toColor());
    }

    public static String getColor(double hue) {
        if(hue <= 40){
            return("red");
        }
        else if (hue >= 70 && hue <= 120){
            return("yellow");
        }
        else if (hue >= 180){
            return("blue");
        }
        else{
            return("no color detected");
        }

    }

    public boolean hasBall(int sensor) {
        return sensors[sensor].getNormalizedColors().alpha >= BALL_ALPHA_THRESHOLD;
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