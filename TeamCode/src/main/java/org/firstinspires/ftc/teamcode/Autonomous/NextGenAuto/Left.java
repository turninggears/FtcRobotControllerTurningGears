package org.firstinspires.ftc.teamcode.Autonomous.NextGenAuto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.System.Alliance;

import java.util.ArrayList;
import java.util.List;


public class Left extends BioBuzzAutonomous {

    protected Left(Alliance alliance) {
        super(alliance, new Pose2d(-60, -60, Math.toRadians(180)));
    }
    @Disabled
    @Autonomous(name = "Left Red", group = "Autonomous")
    public static class LeftRed extends Left {
        public LeftRed() {
            super(Alliance.RED);
        }
    }

    @Override
    public void initialize() {

    }

    public void runAuto() {
        List<Action> actions = new ArrayList<>();
    }
}
