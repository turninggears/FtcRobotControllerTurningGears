
package org.firstinspires.ftc.teamcode.System;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import java.util.List;

@Config
public class Vision {

    // ========================================
    // VISION VARIABLES
    // ========================================

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ========================================
    // INITIALIZATION
    // ========================================

    public Vision(HardwareMap hardwareMap) {

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(
                        hardwareMap.get(WebcamName.class, "Front Camera")
                )
                .addProcessor(aprilTag)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    // ========================================
    // APRILTAG DETECTION
    // ========================================

    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public AprilTagClusterDetection getCluster(String clusterName) {

        for (AprilTagDetection detection : aprilTag.getDetections()) {

            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                if (cluster.metadata != null &&
                        cluster.metadata.name.equalsIgnoreCase(clusterName)) {
                    return cluster;
                }
            }
        }

        return null;
    }

    public boolean isClusterVisible(String clusterName) {
        return getCluster(clusterName) != null;
    }


    // ========================================
    // BIOBUZZ LAUNCHER AIMING
    // ========================================

    // TEMPORARY MEASUREMENTS - VERIFY ON ROBOT

    // Camera and launcher both face rear of robot.
// Positive = camera is farther toward robot's FRONT
// than the pollen release point.
// Negative = camera is farther toward robot's REAR.
    public static double CAMERA_BEHIND_LAUNCHER_IN = 4.0;

    // Positive = camera right of launcher
    // Negative = camera left of launcher
    public static double CAMERA_RIGHT_OFFSET_IN = 0.0;

    // For future vertical aiming calculations
    public static double CAMERA_ABOVE_LAUNCHER_IN = 3.0;

    // Fine adjustment for launcher alignment
    public static double AIM_TRIM_DEGREES = 0.0;

    // Acceptable aiming error
    public static double AIM_TOLERANCE_DEGREES = 2.0;

    // Maximum correction allowed
    public static double MAX_AIM_CORRECTION_DEGREES = 30.0;

    // ========================================
    // CALCULATE AIM CORRECTION
    // ========================================

    public Double getAimCorrection(String clusterName) {

        if (!isCameraStreaming()) {
            return null;
        }

        AprilTagClusterDetection cluster =
                getCluster(clusterName);

        if (cluster == null || cluster.ftcPose == null) {
            return null;
        }

        // Only use upright/scorable clusters
        if (Math.abs(cluster.ftcPose.roll) >= 90.0) {
            return null;
        }

        // FTC camera coordinates:
// X = right/left relative to camera
// Z = forward distance in camera viewing direction
        double targetX = cluster.ftcPose.x;
        double targetZ = cluster.ftcPose.z;

        // Adjust for camera mounting position
        double launcherTargetX =
                targetX + CAMERA_RIGHT_OFFSET_IN;

        double launcherTargetZ =
                targetZ + CAMERA_BEHIND_LAUNCHER_IN;

        if (launcherTargetZ <= 0.0) {
            return null;
        }

        // Camera and launcher both face rear.
// Calculate angle toward target relative to launcher.
// Positive angle means target is to camera's left.
        double correctionDegrees = Math.toDegrees(
                Math.atan2(
                        -launcherTargetX,
                        launcherTargetZ
                )
        ) + AIM_TRIM_DEGREES;

        if (!Double.isFinite(correctionDegrees)) {
            return null;
        }

        if (Math.abs(correctionDegrees)
                > MAX_AIM_CORRECTION_DEGREES) {
            return null;
        }

        return correctionDegrees;
    }

    // ========================================
    // CHECK AIM ALIGNMENT
    // ========================================

    public boolean isAimedAtTarget(String clusterName) {

        Double correction = getAimCorrection(clusterName);

        return correction != null
                && Math.abs(correction)
                <= AIM_TOLERANCE_DEGREES;
    }



    // ========================================
    // CAMERA STATUS
    // ========================================

    public boolean isCameraStreaming() {
        return visionPortal.getCameraState()
                == VisionPortal.CameraState.STREAMING;
    }

    // ========================================
    // STOP / CLEANUP
    // ========================================

    public void close() {
        FtcDashboard.getInstance().stopCameraStream();
        visionPortal.close();
    }
}
