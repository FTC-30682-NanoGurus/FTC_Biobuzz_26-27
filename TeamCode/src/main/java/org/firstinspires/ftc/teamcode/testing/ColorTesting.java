package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


public class ColorTesting extends TestingOpMode {
    ColorSensor test_color;

    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        test_color = hardwareMap.get(ColorSensor.class, "test_color");

        // Wait for the Play button to be pressed

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
            telemetry.addData("Red", test_color.red());
            telemetry.addData("Green", test_color.green());
            telemetry.addData("Blue", test_color.blue());
            telemetry.update();
        }

    }
}
