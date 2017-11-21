package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

/**
 * Created by Michael on 11/16/2017.
 */

@TeleOp(name="ColorSensor", group="Sensor")
public class BallColorSensor extends OpMode {

    private ColorSensor ballColorSensor;



    @Override
    public void init() {
        ballColorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {

        ballColorSensor.enableLed(true);
        telemetry.addData("RGB", formatRGBtoString(getColorSensorRGB(ballColorSensor)));
        telemetry.addData("HUE", getColorSensorHSV(ballColorSensor)[0]);
        telemetry.addData("Color", getBallColor());
        telemetry.update();

    }

    // Get HSV from cs
    private float[] getColorSensorHSV(ColorSensor cs) {
        float hsvValues[] = {0F,0F,0F};
        int[] sensorRGB = getColorSensorRGB(cs);

        Color.RGBToHSV(sensorRGB[0], sensorRGB[1], sensorRGB[2], hsvValues);
        return hsvValues;
    }

    // Get RGB from cs
    private int[] getColorSensorRGB(ColorSensor cs) {
        int rgbValues[] = {0,0,0};
        try {
            rgbValues = new int[]{cs.red(), cs.green(), cs.blue()};
        } catch (Exception ex) {}
        return rgbValues;
    }

    // Format RGB into a string
    private String formatRGBtoString(int[] RGB) {
        String formattedString = "";
        formattedString += Integer.toString(RGB[0]) + "," + Integer.toString(RGB[1]) + "," + Integer.toString(RGB[2]);
        return formattedString;
    }

    private String getBallColor() {
        float hue = getColorSensorHSV(ballColorSensor)[0];
        int[] rgb = getColorSensorRGB(ballColorSensor);

        if (rgb[0] > rgb[1] && rgb[0] > rgb[2]) {
            if (hue > 0 && hue < 10) {
                return "red";
            }
        }

        if (rgb[2] > rgb[0] && rgb[2] > rgb[1]) {
            if (hue > 200 && hue < 255) {
                return "blue";
            }
        }

        return "none detected";
    }

    @Override
    public void stop() {

    }
}
