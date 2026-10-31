package org.firstinspires.ftc.teamcode.System;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class Pause implements Action {

    double seconds;
    long startTime = -1;
    public Pause(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (startTime < 0) {
            startTime = System.currentTimeMillis();
        }
        return System.currentTimeMillis() < startTime + (seconds * 1000);
    }

    public static Action pause() {
        return new Pause(0.5);
    }

}
