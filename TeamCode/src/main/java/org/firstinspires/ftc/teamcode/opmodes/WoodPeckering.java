package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

@Disabled
@Config
@Autonomous
public class WoodPeckering extends NGAutoOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        initAuto(beginPose);
        FailoverAction sleep = new FailoverAction(new SleepAction(0.4), new NullAction());
        FailoverAction armDown2 = new FailoverAction(intake.moveArmFast(200, -0.3), new InstantAction(() -> intake.arm.setManualPower(0)));


        intake.openClaw();
        waitForStart();



        Action pickup = new SequentialAction(
                intake.armAction(500),
                intake.slideAction(700),
                new ParallelAction(
                        new SequentialAction(armDown2, new InstantAction(sleep::failover)),
                        new SequentialAction(sleep, new InstantAction(armDown2::failover))
                ));
        Actions.runBlocking(
                new ParallelAction(bulkRead.update(), intake.updateAction(), pickup ));








    }
}