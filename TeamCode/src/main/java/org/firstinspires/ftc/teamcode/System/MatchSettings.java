
package org.firstinspires.ftc.teamcode.System;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MatchSettings {

    // Available alliance colors
    public enum Alliance {
        RED,
        BLUE
    }

    // Default alliance if none has been selected
    public static Alliance alliance = Alliance.RED;

    // Set alliance from Autonomous
    public static void setAlliance(Alliance selectedAlliance) {
        if (selectedAlliance != null) {
            alliance = selectedAlliance;
        }
    }

    // Get currently selected alliance
    public static Alliance getAlliance() {
        return alliance;
    }

    // Check alliance
    public static boolean isRed() {
        return alliance == Alliance.RED;
    }

    public static boolean isBlue() {
        return alliance == Alliance.BLUE;
    }

    // Get alliance-specific AprilTag cluster names
    public static String getAudienceClusterName() {
        return isRed() ? "RED AUDIENCE" : "BLUE AUDIENCE";
    }

    public static String getScoringClusterName() {
        return isRed() ? "RED SCORING" : "BLUE SCORING";
    }

    // Determine whether a detected cluster belongs
    // to our currently selected alliance
    public static boolean isAllianceCluster(String clusterName) {

        if (clusterName == null) {
            return false;
        }

        return clusterName.equalsIgnoreCase(
                getAudienceClusterName()
        ) || clusterName.equalsIgnoreCase(
                getScoringClusterName()
        );
    }
}
