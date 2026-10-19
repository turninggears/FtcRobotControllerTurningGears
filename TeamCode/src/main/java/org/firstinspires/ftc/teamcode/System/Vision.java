
package org.firstinspires.ftc.teamcode.System;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import com.acmerobotics.dashboard.FtcDashboard;


import java.util.List;

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
