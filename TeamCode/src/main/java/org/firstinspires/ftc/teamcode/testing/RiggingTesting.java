package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DECODE_subsystems.Rigging;

@Config
@TeleOp
public class RiggingTesting extends TestingOpMode {
    Rigging rigging;
    public static int target_position = 0;

    private int past_target_position = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        rigging = new Rigging(hardwareMap, telemetry);

        rigging.init();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addLine("Gamepad1 a to reset rigging if it is higher than hardstop");
            if(gamepad1.a){
                rigging.reset();
            }
            rigging.rigging_motor.setManualPower(-gamepad1.left_stick_y);
            if (target_position != past_target_position) {
                rigging.rigging_motor.move_async(target_position);
            }
            past_target_position = target_position;

            rigging.update();
            rigging.telemetry();
            telemetry.update();

        }
    }
}
