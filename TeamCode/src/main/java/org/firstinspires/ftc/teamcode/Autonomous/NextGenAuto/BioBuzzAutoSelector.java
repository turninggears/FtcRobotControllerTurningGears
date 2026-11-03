
package org.firstinspires.ftc.teamcode.Autonomous.NextGenAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.System.MatchSettings;

@Autonomous(name = "BioBuzz Autonomous", group = "NextGen")
public class BioBuzzAutoSelector extends LinearOpMode {

    // Alliance selection
    private MatchSettings.Alliance selectedAlliance =
            MatchSettings.Alliance.RED;

    // Starting position selection
    private enum StartPosition {
        RIGHT_A,
        LEFT_A
    }

    private StartPosition selectedPosition = StartPosition.RIGHT_A;

    // Previous button states for detecting individual presses
    private boolean previousLeft = false;
    private boolean previousRight = false;
    private boolean previousUp = false;
    private boolean previousDown = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------------
        // INITIALIZATION / SELECTION
        // -------------------------------

        while (opModeInInit() && !isStopRequested()) {

            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            // Select alliance
            if (left && !previousLeft) {
                selectedAlliance = MatchSettings.Alliance.RED;
            }

            if (right && !previousRight) {
                selectedAlliance = MatchSettings.Alliance.BLUE;
            }

            // Select starting position
            if (up && !previousUp) {
                selectedPosition = StartPosition.RIGHT_A;
            }

            if (down && !previousDown) {
                selectedPosition = StartPosition.LEFT_A;
            }

            // Save button states
            previousLeft = left;
            previousRight = right;
            previousUp = up;
            previousDown = down;

            // Driver Station telemetry
            telemetry.addLine("=== BIOBUZZ AUTONOMOUS ===");
            telemetry.addLine();

            telemetry.addData("Alliance", selectedAlliance);
            telemetry.addData("Starting Position", selectedPosition);

            telemetry.addLine();
            telemetry.addLine("GAMEPAD 1 CONTROLS");
            telemetry.addLine("D-Pad LEFT  = RED Alliance");
            telemetry.addLine("D-Pad RIGHT = BLUE Alliance");
            telemetry.addLine("D-Pad UP    = RIGHT A");
            telemetry.addLine("D-Pad DOWN  = LEFT A");

            telemetry.addLine();
            telemetry.addLine("Ready - Press START to begin");
            telemetry.update();

            sleep(50);
        }

        if (isStopRequested()) {
            return;
        }

        // -------------------------------
        // AUTONOMOUS START
        // -------------------------------

        // Save the selected alliance for other robot systems
        MatchSettings.setAlliance(selectedAlliance);

        telemetry.clearAll();
        telemetry.addLine("=== AUTONOMOUS STARTED ===");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Starting Position", selectedPosition);
        telemetry.addLine();
        telemetry.addLine("Path execution not yet implemented.");
        telemetry.update();

        // Temporary test loop
        // This will be replaced with our shared autonomous runner.
        while (opModeIsActive()) {
            idle();
        }
    }
}
