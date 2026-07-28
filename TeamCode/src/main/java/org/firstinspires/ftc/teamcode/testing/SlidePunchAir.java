package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;

@TeleOp
@Config
public class SlidePunchAir extends TestingOpMode {
    Intake intake;
    public static int bottom = 0;
    public static int top = 1200;
    public static int arm_pos = 1400;
    private boolean up = false;

    private boolean prev_a = false;
    private boolean pause = false;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        intake.arm.setUseMotionProfile(false);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if (!pause){
                intake.moveArm(arm_pos);
                if(!intake.arm.isBusy()) {
                    telemetry.addLine("Slides go brr");
                    if (!intake.slides.isBusy()) {
                        intake.moveSlides(up ? bottom : top);
                        up = !up;
                    }
                }
            }
            pause = (gamepad1.a && !prev_a) != pause;
            prev_a = gamepad1.a;
            telemetry.addLine("Press A to pause");
            intake.update();
            intake.telemetry();
            telemetry.update();
        }

    }
}
