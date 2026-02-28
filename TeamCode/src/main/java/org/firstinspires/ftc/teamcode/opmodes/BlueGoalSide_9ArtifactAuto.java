package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

@Disabled
@Autonomous
public class BlueGoalSide_9ArtifactAuto extends NGAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-52, -48 , Math.toRadians(55));
        initAuto(beginPose);

        TrajectoryActionBuilder moveBackwardPath = drive.actionBuilder(beginPose)
                .lineToY(-15, new TranslationalVelConstraint(60));
        TrajectoryActionBuilder PathToFirstSet = moveBackwardPath.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-15, -24), Math.toRadians(90));
        TrajectoryActionBuilder collectFirstSet = drive.actionBuilder(new Pose2d(-15, -24, Math.toRadians(90)))
                .lineToY(-52, new TranslationalVelConstraint(42));
        TrajectoryActionBuilder PathToGoal = collectFirstSet.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-25, -10), Math.toRadians(55), new TranslationalVelConstraint(50));
        TrajectoryActionBuilder PathToSecondSet = PathToGoal.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(7.5, -24, Math.toRadians(90)), Math.toRadians(-20), new TranslationalVelConstraint(40));
        TrajectoryActionBuilder collectSecondSet = drive.actionBuilder(new Pose2d(7.5, -24, Math.toRadians(90)))
                .lineToY(-57.5, new TranslationalVelConstraint(40));
        TrajectoryActionBuilder PathToGoal2 = collectSecondSet.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-25, -10), Math.toRadians(55), new TranslationalVelConstraint(50));
        TrajectoryActionBuilder leaveLaunchLine = PathToGoal2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(5, -10), new TranslationalVelConstraint(70));

        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action scorePreLoaded = moveBackwardPath.build();
        Action goToFirstSet = PathToFirstSet.build();
        Action CollectFirstSet = collectFirstSet.build();
        Action goToGoal = PathToGoal.build();
        Action goToSecondSet = PathToSecondSet.build();
        Action CollectSecondSet = collectSecondSet.build();
        Action goToGoal2 = PathToGoal2.build();
        Action leave = leaveLaunchLine.build();

        telemetry.addLine("Paths Built");
        telemetry.update();

        double shooterTargetVel = DECODERobotConstants.closeZoneShootingVel;
        double hoodShootingPos = DECODERobotConstants.closeShootPos;

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                    bulkRead.update(),
                    intake2_0.updateFlywheelPID(),
                        new ParallelAction(
                        intake2_0.runShooter(shooterTargetVel, 29),
                    new SequentialAction(
                            new ParallelAction(
                                    //intake2_0.runShooter(shooterTargetVel, 2),
                                    intake2_0.setHoodAdjuster(hoodShootingPos),
                                    scorePreLoaded
                            ),
                            new ParallelAction(
                                    //intake2_0.shoot(3, shooterTargetVel),
                                    intake2_0.transferUsingRollersForTime(2.5, 1)
                            ),
                            new SequentialAction(
                                    new SequentialAction(
                                    intake2_0.disableTransfer()
                                    ),
                                    goToFirstSet
                            ),
                            new ParallelAction(
                                intake2_0.collect(1),
                                CollectFirstSet
                            ),
                            new ParallelAction(
                                goToGoal,
                                    //intake2_0.runShooter(shooterTargetVel, 6),
                                    intake2_0.setHoodAdjuster(hoodShootingPos),
                                    intake2_0.preventEscape(0.2)
                            ),
                            new ParallelAction(
                                    intake2_0.transferUsingRollersForTime(3, 1)
                                    //intake2_0.shoot(4.5, shooterTargetVel)
                            ),
                            new ParallelAction(
                                    new SequentialAction(
                                    intake2_0.disableTransfer()
                                    ),
                                    goToSecondSet
                            ),
                            new ParallelAction(
                                    intake2_0.collect(1),
                                    CollectSecondSet
                            ),
                            new ParallelAction(
                                    goToGoal2,
                                        //intake2_0.runShooter(shooterTargetVel, 6),
                                        intake2_0.setHoodAdjuster(hoodShootingPos),
                                    intake2_0.preventEscape(0.2)
                            ),
                            new ParallelAction(
                                    intake2_0.transferUsingRollersForTime(3, 1)
                                    //intake2_0.shoot(4.5, shooterTargetVel)
                            ),
                            new ParallelAction(
                                    intake2_0.disableTransfer()
                                    //intake2_0.stopShooter()
                            ),
                            new SequentialAction(
                                    leave
                            )
                    )
                    )
                )
        );
    }
}
