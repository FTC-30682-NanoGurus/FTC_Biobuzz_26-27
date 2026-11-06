package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

@Config
@Autonomous
public class Biobuzz_Park_Auto extends NGAutoOpMode {

    public static double START_X = -60;
    public static double FORWARD_IN = 2.8;
    public static double PARK_X = -25; // -40

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(START_X, -63, Math.toRadians(90));
        initAuto(beginPose);

        double parkY = beginPose.position.y + FORWARD_IN;
        Action parkPath = drive.actionBuilder(beginPose)
                .lineToY(parkY)
                .strafeTo(new Vector2d(PARK_X, parkY))
                .build();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Park Auto - Path Built - Ready To Start");
            telemetry.addData("Start", "(%.1f, %.1f) facing forward, back touching the wall", beginPose.position.x, beginPose.position.y);
            telemetry.addData("Park", "(%.1f, %.1f)", PARK_X, parkY);
            telemetry.update();
        }
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        bulkRead.update(),
                        parkPath
                )
        );
    }
}
