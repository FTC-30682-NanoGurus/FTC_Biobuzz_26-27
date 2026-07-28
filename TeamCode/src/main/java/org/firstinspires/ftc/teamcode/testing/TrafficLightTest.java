package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.TrafficLight;

@Config
@TeleOp
public class TrafficLightTest extends TestingOpMode {

    TrafficLight trafficLight;
    public static int blinks = 0;
    public static boolean flash = false;

    public static double delay = 0.1;

    public static boolean green = false;
    public static boolean red = false;
    public static boolean offred = false;

    @Override
    public void runOpMode() {
        makeTelemetry();
        trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);
        // Wait for the start button
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // Start the opmode loop
        while (opModeIsActive()) {
            if(flash){
                if(red){
                    trafficLight.flashRed(delay, blinks);
                }
                if(green){
                    trafficLight.flashGreen(delay, blinks);
                }
                if(offred){
                    trafficLight.flashOffred(delay, blinks);
                }
            }
            flash = false;
            trafficLight.update();
        }
    }

}