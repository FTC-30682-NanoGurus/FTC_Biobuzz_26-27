package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
@TeleOp
public class trafficLightTest extends LinearOpMode{

    private RevBlinkinLedDriver trafficLight2_0;
    @Override
    public void runOpMode() throws InterruptedException {
        trafficLight2_0 = hardwareMap.get(RevBlinkinLedDriver.class, "trafficLight2_0");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                trafficLight2_0.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            if (gamepad1.b){
                trafficLight2_0.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        }
    }
}