
package org.firstinspires.ftc.teamcode.System;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@Config
public class Vision {

    // ========================================
    // VISION VARIABLES
    // ========================================

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ========================================
    // ROBOT AND TARGET MEASUREMENTS
    // ========================================

    // Camera is 4 inches behind the pollen
    // release point, toward the robot's rear.
    public static double CAMERA_BEHIND_LAUNCHER_IN = 4.0;

    // Camera lens is 1 inch right of the
    // release point when facing the AprilTags.
    public static double CAMERA_RIGHT_OFFSET_IN = 1.0;

    // Camera is 7 inches above release point.
    public static double CAMERA_ABOVE_LAUNCHER_IN = 7.0;

    // Release point is 7 inches above floor.
    public static double LAUNCHER_HEIGHT_IN = 7.0;

    // Hive pollen scoring area heights.
    public static double TARGET_BOTTOM_HEIGHT_IN = 53.5;
    public static double TARGET_TOP_HEIGHT_IN = 65.6;

    // Aiming adjustments
    public static double AIM_TRIM_DEGREES = 0.0;
    public static double AIM_TOLERANCE_DEGREES = 2.0;
    public static double MAX_AIM_CORRECTION_DEGREES = 30.0;

    // ========================================
    // INITIALIZATION
    // ========================================

    public Vision(HardwareMap hardwareMap) {

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(
                        AprilTagGameDatabase.getBioBuzzTagLibrary()
                )
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(
                        hardwareMap.get(
                                WebcamName.class,
                                "Front Camera"
                        )
                )
                .addProcessor(aprilTag)
                .build();

        FtcDashboard.getInstance()
                .startCameraStream(visionPortal, 30);
    }

    // ========================================
    // ALL APRILTAG DETECTIONS
    // ========================================

    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    // ========================================
    // FIND A SPECIFIC CLUSTER
    // ========================================

    public AprilTagClusterDetection getCluster(
            String clusterName) {

        if (!isCameraStreaming() || clusterName == null) {
            return null;
        }

        for (AprilTagDetection detection : getDetections()) {

            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }

            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) detection;

            if (cluster.metadata != null
                    && cluster.metadata.name != null
                    && cluster.metadata.name.equalsIgnoreCase(
                    clusterName)) {

                return cluster;
            }
        }

        return null;
    }

    public boolean isClusterVisible(String clusterName) {
        return getCluster(clusterName) != null;
    }

    // ========================================
    // FIND CURRENT ALLIANCE CLUSTER
    // ========================================

    public AprilTagClusterDetection getAllianceCluster() {

        if (!isCameraStreaming()) {
            return null;
        }

        for (AprilTagDetection detection : getDetections()) {

            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }

            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) detection;

            if (cluster.metadata == null
                    || cluster.metadata.name == null) {
                continue;
            }

            // Only accept our current alliance.
            // Opposing alliance clusters are ignored.
            if (MatchSettings.isAllianceCluster(
                    cluster.metadata.name)) {

                return cluster;
            }
        }

        return null;
    }

    // ========================================
    // ALLIANCE CLUSTER DETECTION STATUS
    // ========================================

    // Useful for Autonomous conditional logic.
    // This only confirms detection, not hive
    // orientation or scoring availability.

    public boolean isAllianceClusterVisible() {
        return getAllianceCluster() != null;
    }

    public String getAllianceClusterName() {

        AprilTagClusterDetection cluster =
                getAllianceCluster();

        if (cluster == null || cluster.metadata == null) {
            return "NONE";
        }

        return cluster.metadata.name;
    }

    // ========================================
    // VALID ALLIANCE TARGET FOR AIMING
    // ========================================

    private AprilTagClusterDetection getValidAimCluster() {

        AprilTagClusterDetection cluster =
                getAllianceCluster();

        if (cluster == null || cluster.ftcPose == null) {
            return null;
        }

        // Retain the existing roll check.
        if (!Double.isFinite(cluster.ftcPose.roll)
                || Math.abs(cluster.ftcPose.roll) >= 90.0) {
            return null;
        }

        if (!Double.isFinite(cluster.ftcPose.x)
                || !Double.isFinite(cluster.ftcPose.y)
                || !Double.isFinite(cluster.ftcPose.z)) {
            return null;
        }

        return cluster;
    }

    // ========================================
    // TARGET POSITION RELATIVE TO LAUNCHER
    // ========================================

    // FTC camera coordinates:
    // X = right of camera
    // Y = forward from camera
    // Z = upward from camera
    //
    // Camera and launcher both face the rear
    // of the robot.
    //
    // Camera is right of release point by 1".
    // Camera is behind release point by 4".
    // Camera is above release point by 7".

    public Double getLauncherTargetX() {

        AprilTagClusterDetection cluster =
                getValidAimCluster();

        if (cluster == null) {
            return null;
        }

        return cluster.ftcPose.x
                + CAMERA_RIGHT_OFFSET_IN;
    }

    public Double getLauncherTargetY() {

        AprilTagClusterDetection cluster =
                getValidAimCluster();

        if (cluster == null) {
            return null;
        }

        // Release point is 4 inches closer
        // to the target than the camera.
        return cluster.ftcPose.y
                - CAMERA_BEHIND_LAUNCHER_IN;
    }

    public Double getLauncherTargetZ() {

        AprilTagClusterDetection cluster =
                getValidAimCluster();

        if (cluster == null) {
            return null;
        }

        return cluster.ftcPose.z
                + CAMERA_ABOVE_LAUNCHER_IN;
    }

    // ========================================
    // HORIZONTAL DISTANCE TO HIVE
    // ========================================

    // Distance from pollen release point to
    // the detected cluster reference point,
    // measured along the floor plane.

    public Double getLauncherDistance() {

        Double x = getLauncherTargetX();
        Double y = getLauncherTargetY();

        if (x == null || y == null || y <= 0.0) {
            return null;
        }

        return Math.hypot(x, y);
    }

    // ========================================
    // DIRECT 3D DISTANCE TO CLUSTER
    // ========================================

    public Double getLauncherDistance3D() {

        Double horizontal = getLauncherDistance();
        Double z = getLauncherTargetZ();

        if (horizontal == null || z == null) {
            return null;
        }

        return Math.hypot(horizontal, z);
    }

    // ========================================
    // TARGET HEIGHT INFORMATION
    // ========================================

    public double getTargetCenterHeight() {

        return (TARGET_BOTTOM_HEIGHT_IN
                + TARGET_TOP_HEIGHT_IN) / 2.0;
    }

    public double getTargetHeightAboveLauncher() {

        return getTargetCenterHeight()
                - LAUNCHER_HEIGHT_IN;
    }

    // ========================================
    // CALCULATE AUTOMATIC AIM CORRECTION
    // ========================================

    // Positive = target is toward camera-left.
    // Negative = target is toward camera-right.
    //
    // This is a correction relative to the
    // rear-facing launcher, not an absolute
    // Road Runner field heading.

    public Double getAllianceAimCorrection() {

        Double targetX = getLauncherTargetX();
        Double targetY = getLauncherTargetY();

        if (targetX == null
                || targetY == null
                || targetY <= 0.0) {
            return null;
        }

        double correctionDegrees =
                Math.toDegrees(
                        Math.atan2(-targetX, targetY)
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
    // PRESERVE EXISTING AIMING METHOD
    // ========================================

    public Double getAimCorrection(String clusterName) {

        // Only permit clusters matching the
        // currently selected alliance.
        if (!MatchSettings.isAllianceCluster(clusterName)) {
            return null;
        }

        AprilTagClusterDetection cluster =
                getCluster(clusterName);

        if (cluster == null || cluster.ftcPose == null) {
            return null;
        }

        if (!Double.isFinite(cluster.ftcPose.roll)
                || Math.abs(cluster.ftcPose.roll) >= 90.0) {
            return null;
        }

        double targetX = cluster.ftcPose.x
                + CAMERA_RIGHT_OFFSET_IN;

        double targetY = cluster.ftcPose.y
                - CAMERA_BEHIND_LAUNCHER_IN;

        if (!Double.isFinite(targetX)
                || !Double.isFinite(targetY)
                || targetY <= 0.0) {
            return null;
        }

        double correctionDegrees =
                Math.toDegrees(
                        Math.atan2(-targetX, targetY)
                ) + AIM_TRIM_DEGREES;

        if (!Double.isFinite(correctionDegrees)
                || Math.abs(correctionDegrees)
                > MAX_AIM_CORRECTION_DEGREES) {
            return null;
        }

        return correctionDegrees;
    }

    // ========================================
    // CHECK AIM ALIGNMENT
    // ========================================

    public boolean isAllianceAimed() {

        Double correction = getAllianceAimCorrection();

        return correction != null
                && Math.abs(correction)
                <= AIM_TOLERANCE_DEGREES;
    }

    public boolean isAimedAtTarget(String clusterName) {

        Double correction =
                getAimCorrection(clusterName);

        return correction != null
                && Math.abs(correction)
                <= AIM_TOLERANCE_DEGREES;
    }

    // ========================================
    // CAMERA STATUS
    // ========================================

    public boolean isCameraStreaming() {

        return visionPortal != null
                && visionPortal.getCameraState()
                == VisionPortal.CameraState.STREAMING;
    }

    // ========================================
    // STOP / CLEANUP
    // ========================================

    public void close() {

        FtcDashboard.getInstance()
                .stopCameraStream();

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
