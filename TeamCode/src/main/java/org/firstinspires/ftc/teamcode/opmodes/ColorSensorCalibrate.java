package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "Color Sensor Calibrate", group = "Test")
public class ColorSensorCalibrate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, "Color");
        sensor.setGain(15);
        waitForStart();

        float[] hsv = new float[3];

        while (opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            Color.RGBToHSV(
                    (int) (colors.red   * 255),
                    (int) (colors.green * 255),
                    (int) (colors.blue  * 255),
                    hsv
            );

            telemetry.addData("Hue  (0-360°)", "%.1f", hsv[0]);
            telemetry.addData("Saturation",    "%.3f", hsv[1]);
            telemetry.addData("Value",         "%.3f", hsv[2]);
            telemetry.addLine("---");
            telemetry.addData("Red",   "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue",  "%.3f", colors.blue);
            telemetry.update();
        }
    }
}