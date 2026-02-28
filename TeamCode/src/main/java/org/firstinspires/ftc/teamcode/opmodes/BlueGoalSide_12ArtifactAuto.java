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

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;

@Autonomous
public class BlueGoalSide_12ArtifactAuto extends NGAutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.resetPose();
/*TrajectoryActionBuilder PathToGate = collectFirstSet.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-7, -44.5), Math.toRadians(270))
                .lineToY(-56, new TranslationalVelConstraint(12));
                    new SleepAction(2);*/
        Pose2d beginPose = new Pose2d(-52, -48 , Math.toRadians(235));
        initAuto(beginPose);

        TrajectoryActionBuilder moveBackwardPath = drive.actionBuilder(beginPose)
                .lineToY(-15, new TranslationalVelConstraint(60));
        TrajectoryActionBuilder PathToFirstSet = moveBackwardPath.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, -21), Math.toRadians(270));
        TrajectoryActionBuilder collectFirstSet = drive.actionBuilder(new Pose2d(-12, -21, Math.toRadians(270)))
                .lineToY(-47.5, new TranslationalVelConstraint(42));

        TrajectoryActionBuilder PathToGoal = collectFirstSet.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-25, -8), Math.toRadians(235), new TranslationalVelConstraint(50));
        TrajectoryActionBuilder PathToSecondSet = PathToGoal.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(13.2, -17, Math.toRadians(270)), Math.toRadians(-20), new TranslationalVelConstraint(40));
        TrajectoryActionBuilder collectSecondSet = drive.actionBuilder(new Pose2d(13.2, -17, Math.toRadians(270)))
                .lineToY(-50, new TranslationalVelConstraint(30));
        TrajectoryActionBuilder PathToGoal2 = collectSecondSet.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-25, -10), Math.toRadians(235), new TranslationalVelConstraint(50));
        TrajectoryActionBuilder PathToThirdSet = PathToGoal2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(34, -15, Math.toRadians(270)), Math.toRadians(3), new TranslationalVelConstraint(50));
        TrajectoryActionBuilder collectThirdSet = drive.actionBuilder(new Pose2d(34, -15, Math.toRadians(270)))
                .lineToY(-50, new TranslationalVelConstraint(40));
        TrajectoryActionBuilder PathToGoal3 = collectThirdSet.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-35, -12), Math.toRadians(246), new TranslationalVelConstraint(50));
        TrajectoryActionBuilder leaveLaunchLine = PathToGoal2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-36, -15), new TranslationalVelConstraint(70));

        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action scorePreLoaded = moveBackwardPath.build();
        Action goToFirstSet = PathToFirstSet.build();
        Action CollectFirstSet = collectFirstSet.build();
        //Action openGate = PathToGate.build();
        Action goToGoal = PathToGoal.build();
        Action goToSecondSet = PathToSecondSet.build();
        Action CollectSecondSet = collectSecondSet.build();
        Action goToGoal2 = PathToGoal2.build();
        Action goToThirdSet = PathToThirdSet.build();
        Action CollectThirdSet = collectThirdSet.build();
        Action goToGoal3 = PathToGoal3.build();
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
                                intake2_0.runShooter(shooterTargetVel, 30),
                                new SequentialAction(
                                        new ParallelAction(
                                                //intake2_0.runShooter(shooterTargetVel, 2),
                                                intake2_0.setHoodAdjuster(hoodShootingPos),
                                                scorePreLoaded
                                        ),
                                        new ParallelAction(
                                                //intake2_0.shoot(3, shooterTargetVel),
                                                intake2_0.transferUsingRollersForTime(2, 1)
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
                                        /*new SequentialAction(
                                                openGate
                                        )*/
                                        new ParallelAction(
                                                goToGoal,
                                                //intake2_0.runShooter(shooterTargetVel, 6),
                                                intake2_0.setHoodAdjuster(hoodShootingPos)
                                                //intake2_0.preventEscape(0.3)
                                        ),
                                        new ParallelAction(
                                                intake2_0.transferUsingRollersForTime(2, 1)
                                                //intake2_0.shoot(4.5, shooterTargetVel)
                                        ),
                                        new ParallelAction(
                                                new SequentialAction(
                                                        intake2_0.disableTransfer()
                                                ),
                                                goToSecondSet
                                        ),
                                        new ParallelAction(
                                                intake2_0.collect(1.35),
                                                CollectSecondSet
                                        ),
                                        new ParallelAction(
                                                goToGoal2,
                                                //intake2_0.runShooter(shooterTargetVel, 6),
                                                intake2_0.setHoodAdjuster(hoodShootingPos)
                                                //intake2_0.preventEscape(0.3)
                                        ),
                                        new ParallelAction(
                                                intake2_0.transferUsingRollersForTime(2, 1)
                                                //intake2_0.shoot(4.5, shooterTargetVel)
                                        ),
                                        new ParallelAction(
                                                intake2_0.disableTransfer(),
                                                goToThirdSet
                                                //intake2_0.stopShooter()
                                        ),
                                        new ParallelAction(
                                                intake2_0.collect(1),
                                                CollectThirdSet
                                        ),
                                        new ParallelAction(
                                                goToGoal3,
                                                //intake2_0.runShooter(shooterTargetVel, 6),
                                                intake2_0.setHoodAdjuster(hoodShootingPos)
                                                //intake2_0.preventEscape(0.3)
                                        ),
                                        new ParallelAction(
                                                intake2_0.transferUsingRollersForTime(2, 1)
                                        ),
                                        new SequentialAction(
                                                //leave,
                                                intake2_0.setFinished()
                                        )
                                )
                        )
                )

        );
        if(isStopRequested() || intake2_0.isAutoFinished()){
            PoseStorage.storeBluePose(drive.pose);
        }
    }
}
