
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorTest extends OpMode {

    private final NormalizedColorSensor[] sensors =
            new NormalizedColorSensor[4];

    private final String[] names = {
            "Color Sensor 0",
            "Color Sensor 1",
            "Color Sensor 2",
            "Color Sensor 3"
    };

    @Override
    public void init() {

        for (int i = 0; i < 4; i++) {

            try {
                sensors[i] = hardwareMap.get(
                        NormalizedColorSensor.class,
                        names[i]
                );

                telemetry.addData(names[i], "Initialized");

            } catch (Exception e) {

                sensors[i] = null;

                telemetry.addData(
                        names[i],
                        "NOT AVAILABLE: " + e.getMessage()
                );
            }
        }

        telemetry.update();
    }

    @Override
    public void loop() {

        for (int i = 0; i < 4; i++) {

            telemetry.addLine("");
            telemetry.addLine("===== " + names[i] + " =====");

            if (sensors[i] == null) {

                telemetry.addData(
                        names[i] + " Status",
                        "NOT AVAILABLE"
                );

                continue;
            }

            try {

                NormalizedRGBA colors =
                        sensors[i].getNormalizedColors();

                double hue =
                        JavaUtil.colorToHue(colors.toColor());

                telemetry.addData(
                        names[i] + " Hue", "%.1f", hue
                );

                telemetry.addData(
                        names[i] + " Red", "%.3f", colors.red
                );

                telemetry.addData(
                        names[i] + " Green", "%.3f", colors.green
                );

                telemetry.addData(
                        names[i] + " Blue", "%.3f", colors.blue
                );

                telemetry.addData(
                        names[i] + " Alpha", "%.3f", colors.alpha
                );

            } catch (Exception e) {

                telemetry.addData(
                        names[i] + " Error",
                        e.getMessage()
                );
            }
        }

        telemetry.update();
    }
}
