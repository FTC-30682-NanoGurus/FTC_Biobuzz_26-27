package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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
public class RedGoalSideAuto extends NGAutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.resetPose();

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < -24.0) {
                return new MinMax(-2,2);
            } else {
                return new MinMax(-100,100);
            }
        };

/*TrajectoryActionBuilder PathToGate = collectFirstSet.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-7, 44.5), Math.toRadians(-270))
                .lineToY(56, new TranslationalVelConstraint(12));
                    new SleepAction(2);*/

        Pose2d beginPose = new Pose2d(-52, 48, Math.toRadians(-235));
        initAuto(beginPose);

        TrajectoryActionBuilder moveBackwardPath = drive.actionBuilder(beginPose)
                .lineToY(20, new TranslationalVelConstraint(45), new ProfileAccelConstraint(-30, 40))
                .afterTime(1.5, intake2_0.transferUsingRollersForTime(1.0, 1));

        TrajectoryActionBuilder ToFirstSet = moveBackwardPath.endTrajectory().fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-22, 13, Math.toRadians(-270)), Math.toRadians(-5), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 60))
                .afterTime(0, intake2_0.collect(0.9))
                .splineToLinearHeading(new Pose2d(-23, 46, Math.toRadians(-270)), Math.toRadians(100), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20, 30))
                .splineToSplineHeading(new Pose2d(-4, 30, Math.toRadians(-270)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-4, 49.2, Math.toRadians(-270)), Math.toRadians(90), new TranslationalVelConstraint(15), new ProfileAccelConstraint(-10, 30))
                .waitSeconds(1.0)
                .splineToSplineHeading(new Pose2d(-26, 20, Math.toRadians(-232)), Math.toRadians(-160), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30, 50));

        TrajectoryActionBuilder ToSecondSet = ToFirstSet.endTrajectory().fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-14.5, 13, Math.toRadians(90)), Math.toRadians(0), new TranslationalVelConstraint(50),  new ProfileAccelConstraint(-10, 60))
                .afterTime(0, intake2_0.collect(1.3))
                .splineToLinearHeading(new Pose2d(15, 49, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(35))
                .splineToSplineHeading(new Pose2d(14, 30, Math.toRadians(90)), Math.toRadians(-90), new TranslationalVelConstraint(50))
                .splineToSplineHeading(new Pose2d(-26, 22, Math.toRadians(130)), Math.toRadians(-180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20, 40));

        TrajectoryActionBuilder ToThirdSet = ToSecondSet.endTrajectory().fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(16, 10, Math.toRadians(90)), Math.toRadians(0), new TranslationalVelConstraint(50),  new ProfileAccelConstraint(-10, 60))
                .afterTime(0, intake2_0.collect(1.2))
                .splineToLinearHeading(new Pose2d(30, 50, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(35))
                .splineToSplineHeading(new Pose2d(28, 35, Math.toRadians(90)), Math.toRadians(-90), new TranslationalVelConstraint(50))
                .splineToSplineHeading(new Pose2d(-26, 22, Math.toRadians(129)), Math.toRadians(-180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20, 40));

        TrajectoryActionBuilder leaveFromLine = ToThirdSet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-2, 36), Math.toRadians(0));

        /*TrajectoryActionBuilder ToFirstGateSet = ToSecondSet.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(8, 47.6, Math.toRadians(-240)), Math.toRadians(120), new TranslationalVelConstraint(40))
                .strafeToConstantHeading(new Vector2d(8, 57));

        TrajectoryActionBuilder ToGoal1 = ToFirstGateSet.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(6, 33, Math.toRadians(-239)), Math.toRadians(-90), new TranslationalVelConstraint(60))
                .splineToSplineHeading(new Pose2d(-26, 22, Math.toRadians(-232)), Math.toRadians(200), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-90, 90))
                .afterTime(0.8, intake2_0.transferUsingRollersForTime(1.0, 1))
                .waitSeconds(0.7)
                .splineToLinearHeading(new Pose2d(8, 47.6, Math.toRadians(-235)), Math.toRadians(120), new TranslationalVelConstraint(40))
                .strafeToConstantHeading(new Vector2d(8, 57));
                //.strafeToLinearHeading(new Vector2d(21, 59), Math.toRadians(-210), new TranslationalVelConstraint(60))
                //.afterTime(0, intake2_0.collect(1.0))
                //.strafeToLinearHeading(new Vector2d(10, 59), Math.toRadians(-210), new TranslationalVelConstraint(42));

        TrajectoryActionBuilder ToGoal2 = ToGoal1.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(6, 33, Math.toRadians(-239)), Math.toRadians(-90), new TranslationalVelConstraint(60))
                .splineToSplineHeading(new Pose2d(-26, 22, Math.toRadians(-232)), Math.toRadians(200), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-90, 90))
                .afterTime(0.8, intake2_0.transferUsingRollersForTime(1.0, 1))
                .waitSeconds(0.7)
                .splineToLinearHeading(new Pose2d(8, 47.6, Math.toRadians(-235)), Math.toRadians(120), new TranslationalVelConstraint(40))
                .strafeToConstantHeading(new Vector2d(8, 57));
                //.strafeToLinearHeading(new Vector2d(21, 59), Math.toRadians(-210), new TranslationalVelConstraint(60))
                //.afterTime(0, intake2_0.collect(1.0))
                //.strafeToLinearHeading(new Vector2d(10, 59), Math.toRadians(-210), new TranslationalVelConstraint(42));
        /*.splineToSplineHeading(new Pose2d(10.5, 33, Math.toRadians(-239)), Math.toRadians(-90), new TranslationalVelConstraint(60))
                .afterTime(1.1, intake2_0.transferUsingRollersForTime(1.2, 1))
                .splineToSplineHeading(new Pose2d(-50, 25, Math.toRadians(-285)), Math.toRadians(200), new TranslationalVelConstraint(60), smartScore)
                .splineToLinearHeading(new Pose2d(9, 40, Math.toRadians(-243)), Math.toRadians(120), new TranslationalVelConstraint(40));*/
        //.strafeToConstantHeading(new Vector2d(9, 58));

        /*TrajectoryActionBuilder ToGoal3 = ToGoal2.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(6, 33, Math.toRadians(-239)), Math.toRadians(-90), new TranslationalVelConstraint(60))
                .splineToSplineHeading(new Pose2d(-26, 22, Math.toRadians(-232)), Math.toRadians(200), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-90, 90))
                .afterTime(0.8, intake2_0.transferUsingRollersForTime(1.0, 1));

        TrajectoryActionBuilder leaveFromGate = ToGoal2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(10, 36), new TranslationalVelConstraint(120));
        */
        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action scorePreLoaded = moveBackwardPath.build();
        Action intakeFirstSet = ToFirstSet.build();
        Action intakeSecondSet = ToSecondSet.build();
        Action intakeThirdSet = ToThirdSet.build();
        Action leave = leaveFromLine.build();
        /*Action toGateSet1 = ToFirstGateSet.build();
        Action ShootToIntake1 = ToGoal1.build();
        Action shootToIntake2 = ToGoal2.build();
        Action shoot3 = ToGoal3.build();
        Action leave = leaveFromGate.build();*/

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
                                                scorePreLoaded,
                                                intake2_0.setHoodAdjuster(0.9)
                                                ),
                                        new SequentialAction(
                                                new ParallelAction(
                                                        intakeFirstSet,
                                                        intake2_0.setHoodAdjuster(0.82)
                                                        ),
                                                intake2_0.transferUsingRollersForTime(1.2, 1)
                                        ),
                                        new SequentialAction(
                                                new ParallelAction(
                                                        intakeSecondSet,
                                                        intake2_0.setHoodAdjuster(0.8)
                                                        ),
                                                intake2_0.transferUsingRollersForTime(1.2, 1)
                                        ),
                                        new SequentialAction(
                                                new ParallelAction(
                                                        intakeThirdSet,
                                                        intake2_0.setHoodAdjuster(0.8)
                                                ),
                                                intake2_0.transferUsingRollersForTime(1.2, 1)
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