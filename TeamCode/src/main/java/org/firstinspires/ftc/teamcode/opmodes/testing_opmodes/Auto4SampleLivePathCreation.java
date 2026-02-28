package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

@Disabled
@Autonomous
public class Auto4SampleLivePathCreation extends NGAutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-32.5, -64, Math.toRadians(0));
        initAuto(beginPose);
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-10,22);
            } else {
                return new MinMax(-30,50);
            }
        };
        FailoverAction pickupAfterDistance2 = new FailoverAction(pickupAfterDistance(RobotConstants.TOO_FAR, intake.distance), new InstantAction(() -> intake.distance.setOn(false)), false);

        FailoverAction pickupAfterDistance3 = new FailoverAction(pickupAfterDistance(RobotConstants.TOO_FAR, intake.distance), new InstantAction(() -> intake.distance.setOn(false)), false);

        FailoverAction pickupAfterDistance4 = new FailoverAction(intake.distance.waitAction(3.7),new InstantAction(() -> intake.distance.setOn(false)), false );


        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));
        TrajectoryActionBuilder firstSamplePath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(135))
                .splineToSplineHeading(basket, Math.toRadians(135), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,40));

        TrajectoryActionBuilder secondSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .afterTime(0.1, new InstantAction(pickupAfterDistance2::enable))
                .splineToSplineHeading(new Pose2d(-48, -43, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,22))
                .splineToSplineHeading(new Pose2d(-48, -35, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-8,10))
                .stopAndAdd(new InstantAction(pickupAfterDistance2::failover));


        FailoverAction firstSample = new FailoverAction(firstSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction secondSample = new FailoverAction( secondSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));


        FailoverAction updateAction1 = updateAction();
        FailoverAction updateAction2 = updateAction();
        FailoverAction updateAction3 = updateAction();
        FailoverAction updateAction4 = updateAction();

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        updateAction1,
                        new SequentialAction(
                                new ParallelAction(
                                        firstSample,
                                        new SequentialAction(
                                                new SleepAction(0.3),
                                                intake.raiseArm(false)
                                        )
                                ),
                                intake.score(),
                                goToSample(secondSample, pickupAfterDistance2),
                                new InstantAction(updateAction1::failover)
                        )

                )
        );

        TrajectoryActionBuilder scoreSecondSamplePath = drive.fastActionBuilder(drive.pose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-54.5, -59, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,30));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd( new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(135))
                .afterTime(0.1, new InstantAction(pickupAfterDistance3::enable))
                .splineToSplineHeading(new Pose2d(-59, -43, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,22))
                .splineToSplineHeading(new Pose2d(-59, -35, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,12))
                .stopAndAdd(new InstantAction(pickupAfterDistance3::failover));



        Action scoreSecondSample = scoreSecondSamplePath.build();
        FailoverAction thirdSample = new FailoverAction(thirdSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));




        Actions.runBlocking(
                new ParallelAction(
                        updateAction2,
                        new SequentialAction(
                                collectSampleAndScore(scoreSecondSample, RobotConstants.claw_floor_pickup, false),
                                goToSample(thirdSample, pickupAfterDistance3)

                        )
                )
        );
        TrajectoryActionBuilder scoreThirdSamplePath = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-55, -59, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(35), new ProfileAccelConstraint(-12,30));

        TrajectoryActionBuilder fourthSamplePath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-52.5, -30.5, Math.toRadians(180)), Math.toRadians(90), new TranslationalVelConstraint(40), pickSampleAccel)
                .afterTime(0.1, new InstantAction(pickupAfterDistance4::enable))
                .splineToConstantHeading(new Vector2d(-61,-27), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10,15))
                .stopAndAdd(new InstantAction(pickupAfterDistance4::failover));

        Action scoreThirdSample = scoreThirdSamplePath.build();
        FailoverAction fourthSample = new FailoverAction( fourthSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));

        Actions.runBlocking(
                new ParallelAction(
                        updateAction3,
                        new SequentialAction(
                                collectSampleAndScore(scoreThirdSample, RobotConstants.claw_open, false),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_open)),
                                intake.armAction(500,800),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_open)),
                                new ParallelAction(fourthSample, intake.armAction(500),
                                        new InstantAction(() -> intake.distance.setOn(true)),
                                        new SequentialAction(intake.slideAction(200),
                                                new InstantAction(() -> intake.turnAndRotateClaw(180,90))
                                        ),
                                        new SequentialAction(
                                                pickupAfterDistance4,
                                                new InstantAction(fourthSample::failover)
                                        )
                                ),
                                new InstantAction(updateAction3::failover)
                        )
                )
        );

        TrajectoryActionBuilder scoreFourthSamplePath = drive.fastActionBuilder(drive.pose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(270), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-12,32))
                .splineToConstantHeading(new Vector2d(-57, -57), Math.toRadians(225), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-12,32));


        Action scoreFourthSample = scoreFourthSamplePath.build();
        Actions.runBlocking(
                new ParallelAction(
                        updateAction4,
                        new SequentialAction(
                                new InstantAction(() -> intake.arm.setExitWithTime(true)),
                                intake.armAction(50),
                                new InstantAction(() -> intake.arm.setExitWithTime(false)),
                                intake.grab(RobotConstants.claw_closed),
                                new ParallelAction(
                                        intake.raiseArmButNotSnagOnBasket(),
                                        scoreFourthSample

                                ),
                                intake.score(true),
                                intake.armAction(0),
                                new InstantAction(updateAction4::failover)
                        )
                )
        );
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
    public FailoverAction updateAction(){
        return new FailoverAction(new ParallelAction(
                intake.distance.updateAction(),
                intake.updateAction()
                //trafficLight.updateAction()
        ),
                new NullAction()
        );
    }
}