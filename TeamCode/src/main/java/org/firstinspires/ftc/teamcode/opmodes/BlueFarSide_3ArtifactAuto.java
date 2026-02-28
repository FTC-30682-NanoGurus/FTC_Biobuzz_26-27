package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

@Autonomous
public class BlueFarSide_3ArtifactAuto extends NGAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(61, -13, Math.toRadians(180));
        initAuto(beginPose);

        //TrajectoryActionBuilder shootingAngle = drive.actionBuilder(beginPose);
                //.lineToXLinearHeading(53, );
        TrajectoryActionBuilder leaveLaunchLine = drive.actionBuilder(beginPose)
            .lineToX(32);

        telemetry.addLine("Ready To Start");
        telemetry.update();

        //Action moveToShoot = shootingAngle.build();
        Action leave = leaveLaunchLine.build();

        telemetry.addLine("Paths Built");
        telemetry.update();

        double shooterTargetVel = DECODERobotConstants.farZoneShootingVel;
        double hoodShootingPos = DECODERobotConstants.farShootPos;

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        bulkRead.update()

                                //intake2_0.runShooter(shooterTargetVel, 6),
                                /*new SequentialAction(
                                        new SequentialAction(
                                                new ParallelAction(
                                                        moveToShoot,
                                                        intake2_0.setHoodAdjuster(hoodShootingPos)
                                                ),
                                                intake2_0.transferUsingRollersForTime(4, 0.65)
                                        )*/,
                                        new SequentialAction(
                                                leave
                                        )
                        )
        );

    }
}
