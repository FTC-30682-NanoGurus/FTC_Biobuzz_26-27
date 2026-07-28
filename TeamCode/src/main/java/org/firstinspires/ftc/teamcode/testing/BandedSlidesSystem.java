package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.DECODE_subsystems.Rigging;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.VihasCameraArm;


@TeleOp
@Config
public class BandedSlidesSystem extends TestingOpMode {
    public static double left_pos_open = 1;
    public static double left_pos_closed = 0;
    public static double right_pos_open = 0;
    public static double right_pos_closed = 1;

    private boolean opened = false;
   Rigging rigging;
    VihasCameraArm vihasCameraArm;

    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        rigging = new Rigging(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        vihasCameraArm = new VihasCameraArm(hardwareMap, telemetry);
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        rigging.openServos();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad2.copy(gamepad2);
            currentGamepad1.copy(gamepad1);

           if(currentGamepad1.a && !previousGamepad1.a){
              rigging.toggleServos();
           }
            rigging.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Motor Power", vihasCameraArm.rigging_motor.getPower());
            telemetry.update();

        }

    }
}
