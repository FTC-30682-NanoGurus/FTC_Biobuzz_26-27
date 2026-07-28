package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Camera;

@Disabled
@Config
@Autonomous
public class AutomaticPickup2 extends NGAutoOpMode {

    Camera camera;
    public static boolean debug = true;
    public static int arm_position = 0;
    public static int slide_position = 0;
    public static double wrist_angle = 0;
    public static double claw_angle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        initAuto(beginPose);

        camera = new Camera(hardwareMap, telemetry);
        camera.init();
        intake.openClaw();
        waitForStart();
        vihasCameraArm.camera();
        camera.startCamera();
        while(opModeIsActive() && !isStopRequested()) {
            bulkRead.update();
            intake.moveArm(arm_position);
            intake.moveSlides(slide_position);
            intake.turnAndRotateClaw(wrist_angle, claw_angle);
            intake.update();
            camera.debug(debug);
        }
    }
}