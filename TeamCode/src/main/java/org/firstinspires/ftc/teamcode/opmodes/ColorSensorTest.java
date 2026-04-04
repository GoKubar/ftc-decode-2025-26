package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorTest extends LinearOpMode {

    private static final String PIN0_NAME = "pin0";
    private static final String PIN1_NAME = "pin1";

    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel pin0 = hardwareMap.digitalChannel.get(PIN0_NAME);
        DigitalChannel pin1 = hardwareMap.digitalChannel.get(PIN1_NAME);

        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Color Sensor Test Ready (Digital Mode)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pin 0", pin0.getState());
            telemetry.addData("Pin 1", pin1.getState());
            telemetry.update();
        }
    }
}
