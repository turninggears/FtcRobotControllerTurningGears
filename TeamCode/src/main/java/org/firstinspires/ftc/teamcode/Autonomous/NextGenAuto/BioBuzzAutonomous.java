package org.firstinspires.ftc.teamcode.Autonomous.NextGenAuto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrivePoseMap;
import org.firstinspires.ftc.teamcode.System.Alliance;
import org.firstinspires.ftc.teamcode.System.Intake;
import org.firstinspires.ftc.teamcode.System.Launcher;

public abstract class BioBuzzAutonomous extends LinearOpMode {

    protected final Alliance alliance;
    protected final Pose2d startingPose;
    protected final Pose2d mappedStartingPose;

    protected MecanumDrivePoseMap drive;
    protected Intake intake;
    protected Launcher launcher;


    protected BioBuzzAutonomous(Alliance alliance, Pose2d startingPose) {
        this.alliance = alliance;
        this.startingPose = startingPose;
        this.mappedStartingPose = alliance.transformPose(startingPose);

    }

    protected PoseMap getPoseMap() {
        return alliance.getPoseMap();
    }

    protected abstract void initialize();
    protected abstract void runAuto();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) {
            return;
        }
        try {
            runAuto();
        } finally {
            String blackboardValue = "mock Blackboard value";
            blackboard.put("mock Blackboard key", blackboardValue);
        }
    }
}
