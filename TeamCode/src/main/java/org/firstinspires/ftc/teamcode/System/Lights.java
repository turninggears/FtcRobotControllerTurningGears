package org.firstinspires.ftc.teamcode.System;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.System.MatchSettings;

public class Lights {

    private final Servo[] lights = new Servo[3];

    private static final double YELLOW = 0.350;
    private static final double GREEN = 0.512;

    private static final double RED = 0.280;
    private static final double BLUE = 0.611;

    public Lights(HardwareMap hardwareMap) {

        for (int i = 0; i < lights.length; i++) {
            String[] names = {
                    "Light1",
                    "Light2",
                    "Light3"
            };
            lights[i] = hardwareMap.get(Servo.class, names[i]);
        }
    }

    public void setColor(MatchSettings.LightColors lightColor, int light) {
        switch (lightColor) {
            case LIGHT_GREEN:
                lights[light].setPosition(GREEN);
                break;
            case LIGHT_YELLOW:
            default:
                lights[light].setPosition(YELLOW);
                break;
        }

    }

    public void setColor(MatchSettings.Alliance alliance, int light) {
        switch (alliance) {
            case BLUE:
                lights[light].setPosition(BLUE);
                break;
            case RED:
            default:
                lights[light].setPosition(RED);
                break;
        }
    }

    public double getColor(int light) {
        return lights[light].getPosition();
    }

    public void lightOff(int light) {
        lights[light].setPosition(0);
    }

    public void lightOn(int light) {
        lights[light].setPosition(1.0);
    }
}
