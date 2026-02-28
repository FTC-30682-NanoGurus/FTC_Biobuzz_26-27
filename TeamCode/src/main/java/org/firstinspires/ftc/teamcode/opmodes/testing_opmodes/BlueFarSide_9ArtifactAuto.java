package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

@Disabled
@Autonomous
public class BlueFarSide_9ArtifactAuto extends NGAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(61, -9 , Math.toRadians(0));
        initAuto(beginPose);
        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -20.0) {
                return new MinMax(-10, 50);
            } else {
                return new MinMax(-30, 50);
            }
        };
        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -32.0) {
                return new MinMax(-20, 80);
            } else {
                return new MinMax(-30, 80);
            }
        };
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-10, 12);
            } else {
                return new MinMax(-30, 50);
            }
        };
        VelConstraint highModeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -25.0) {
                return 20;
            } else {
                return 50;
            }
        };

        TrajectoryActionBuilder moveToGoalPath = drive.actionBuilder(beginPose)
                .splineToSplineHeading(new Pose2d(-45, -28, Math.toRadians(55)), Math.toRadians(-40), new TranslationalVelConstraint(60));
        TrajectoryActionBuilder PathToFirstSet = moveToGoalPath.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-12, -34, Math.toRadians(90)), Math.toRadians(-50), new TranslationalVelConstraint(40));
        TrajectoryActionBuilder collectFirstSet = drive.actionBuilder(new Pose2d(-12, -34, Math.toRadians(90)))
                .lineToY(-53, new TranslationalVelConstraint(13));
        TrajectoryActionBuilder PathToGoal = PathToFirstSet.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-45, -28), Math.toRadians(55), new TranslationalVelConstraint(80));
        TrajectoryActionBuilder PathToSecondSet = PathToGoal.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(10, -34, Math.toRadians(90)), Math.toRadians(-20), new TranslationalVelConstraint(60));
        TrajectoryActionBuilder collectSecondSet = drive.actionBuilder(new Pose2d(10, -34, Math.toRadians(90)))
                .lineToY(-53, new TranslationalVelConstraint(13));

        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action scorePreLoaded = moveToGoalPath.build();
        Action goToFirstSet = PathToFirstSet.build();
        Action CollectFirstSet = collectFirstSet.build();
        Action goToGoal = PathToGoal.build();
        Action goToSecondSet = PathToSecondSet.build();
        Action CollectSecondSet = collectSecondSet.build();

        telemetry.clear();
        telemetry.addLine("Paths Built");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        bulkRead.update(),
                        new SequentialAction(
                                new SequentialAction(
                                        scorePreLoaded,
                                        intake2_0.shoot(4, 1)
                                ),
                                new SequentialAction(
                                        goToFirstSet
                                ),
                                new ParallelAction(
                                        intake2_0.collect(3),
                                        CollectFirstSet
                                ),
                                new SequentialAction(
                                        goToGoal,
                                        intake2_0.shoot(4, 1)
                                ),
                                new SequentialAction(
                                        goToSecondSet
                                ),
                                new ParallelAction(
                                        intake2_0.collect(3),
                                        CollectSecondSet
                                ),
                                new SequentialAction(
                                        goToGoal,
                                        intake2_0.shoot(4, 1)
                                )
                        )
                )
        );
    }
}
