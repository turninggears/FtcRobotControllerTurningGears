package org.firstinspires.ftc.teamcode.System;

import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2dDual;

import java.util.function.Function;

public enum Alliance {
    RED(pose ->
            new Pose2dDual<>(
                    new Vector2dDual<>(
                            pose.position.x.unaryMinus(),
                            pose.position.y.unaryMinus()),
                    pose.heading.inverse()),
            (Pose2d p) -> new Pose2d(
                    -1 * p.position.x,
                    -1 * p.position.y,
                    p.heading.inverse().toDouble()),
            20,
            // Forward (0 degrees field-centric driving) is -90 deg. on
            // the field.
            -90
    ),
    BLUE(
            new IdentityPoseMap(),
            (Pose2d p) -> p,
            24,
            // Forward (0 degrees field-centric driving) is +90 deg. on
            // the field.
            90
    );

    // For RoadRunner trajectories
    final private PoseMap poseMap;

    // For mapping the MecanumDrive's starting pose
    final private Function<Pose2d, Pose2d> poseTransform;

    // What ID is the april tag for this alliance's goal?
    final private int goalAprilTagId;

    // The heading offset in degrees to be used for field-centric driving, relative
    // to standard field coordinates.  Coming from Autonomous, our robot pose will
    // reflect standard field coordinates/orientation.
    final private int headingOffset;

    Alliance(
            PoseMap poseMap,
            Function<Pose2d, Pose2d> poseTransform,
            int goalAprilTagId,
            int headingOffset
    ) {
        this.poseMap = poseMap;
        this.poseTransform = poseTransform;
        this.goalAprilTagId = goalAprilTagId;
        this.headingOffset = headingOffset;
    }

    public PoseMap getPoseMap() {
        return poseMap;
    }

    public Pose2d transformPose(Pose2d p) {
        return poseTransform.apply(p);
    }

    public int getGoalAprilTagId() {
        return goalAprilTagId;
    }

    public int getHeadingOffset() {
        return headingOffset;
    }
}
