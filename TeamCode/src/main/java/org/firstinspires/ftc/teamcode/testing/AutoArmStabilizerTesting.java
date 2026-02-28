package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

@Disabled
public class AutoArmStabilizerTesting extends NGAutoOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(90));
        initAuto(beginPose);



        waitForStart();


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(DISTANCE)
                        .lineToX(0)
                        .build()
        );


    }
}